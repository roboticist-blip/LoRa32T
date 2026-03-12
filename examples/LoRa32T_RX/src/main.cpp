/*
 * Receiver side – ESP32 + SX1278 LoRa
 *
 * Receives encrypted telemetry, decrypts it (using whichever AES mode
 * the transmitter selected), tracks link metrics, and replies with a
 * compact ACKMetrics message so the TX can make its next mode decision.
 *
 * The library is used here purely for its crypto primitives and ACK builder.
 * Mode selection logic lives entirely on the TX.
 */

#include <SPI.h>
#include <LoRa.h>
#include "LoRa32T.h"

// ─── LoRa pin mapping (adjust to your board) ─────────────────────────────────
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 26
#define LED_PIN   13
static const uint32_t LED_BLINK_MS = 80;

static const uint8_t PSK[16] = {
    0xDE,0xAD,0xBE,0xEF, 0xCA,0xFE,0xBA,0xBE,
    0x01,0x23,0x45,0x67, 0x89,0xAB,0xCD,0xEF
};

#define PAYLOAD_SIZE  64
#define HEADER_SIZE   (1 + 16 + 16 + 4)   // mode + iv + tag + seqNum

struct Telemetry {
    uint32_t seqNum;
    float    temperature;
    float    pressure;
    float    latitude;
    float    longitude;
    uint32_t timestamp;
    uint8_t  pad[64 - 4 - 4*4 - 4];
};

#define WINDOW_SIZE 50

struct RxStats {
    uint32_t total;
    uint32_t lost;       
    uint32_t authFails;
    uint32_t lastSeq;
    uint8_t  windowLost[WINDOW_SIZE];
    uint8_t  windowAuth[WINDOW_SIZE];
    uint8_t  head;       
};

LoRa32T sec;
RxStats         stats = {};
bool ledPulseActive = false;
uint32_t ledOffAtMs = 0;

static constexpr float COST_ALPHA  = 0.55f;
static constexpr float COST_BETA   = 0.30f;
static constexpr float COST_GAMMA0 = 0.10f;
static constexpr float COST_KAPPA  = 0.25f;
static constexpr float TOA_NORM[3] = {0.80f, 1.00f, 0.95f};
static constexpr SecurityPolicy COST_DEBUG_POLICY = SecurityPolicy::BestEffort;

const char* modeName(AESMode m)
{
    switch (m) {
        case AESMode::CTR: return "CTR";
        case AESMode::CBC: return "CBC";
        case AESMode::GCM: return "GCM";
    }
    return "UNK";
}

float retryFactor(float per)
{
    float p = (per > 0.95f) ? 0.95f : per;
    return 1.0f / (1.0f - p);
}

float computeDebugCost(AESMode mode,
                       SecurityPolicy policy,
                       float snr,
                       float perRadio,
                       float perAuth,
                       float battLevel)
{
    if (policy == SecurityPolicy::Mandatory && mode != AESMode::GCM)
        return 1e9f;
    if (policy == SecurityPolicy::Conditional && mode == AESMode::CTR && snr > 0.0f)
        return 1e9f;

    const uint8_t idx = static_cast<uint8_t>(mode);
    float perEff = perRadio;
    if (mode == AESMode::GCM)
        perEff += perAuth;

    const float fLink    = perEff;
    const float fLatency = TOA_NORM[idx] * retryFactor(perEff);
    const float fEnergy  = TOA_NORM[idx];
    const float gammaEff = COST_GAMMA0 + COST_KAPPA * (1.0f - battLevel);
    return COST_ALPHA * fLink + COST_BETA * fLatency + gammaEff * fEnergy;
}

void updateWindowStat(uint8_t* win, uint8_t& head, uint8_t val,
                       uint32_t& total, uint32_t& counter)
{
    counter -= win[head];
    win[head] = val;
    counter  += val;
    head = (head + 1) % WINDOW_SIZE;
    total++;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6)) {
        Serial.println("[RX] LoRa init failed");
        while (true);
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();

    // Library used only for crypto + ACK building on RX side
    // No cost weights needed; supply a dummy struct
    sec.begin(PSK);

    memset(&stats, 0, sizeof(stats));
    Serial.println("[RX] Listening");
}

void loop()
{
    if (ledPulseActive && (int32_t)(millis() - ledOffAtMs) >= 0) {
        digitalWrite(LED_PIN, LOW);
        ledPulseActive = false;
    }

    int pktSize = LoRa.parsePacket();
    if (pktSize < (int)(HEADER_SIZE + PAYLOAD_SIZE)) return;

    digitalWrite(LED_PIN, HIGH);
    ledPulseActive = true;
    ledOffAtMs = millis() + LED_BLINK_MS;

    // ── 1. Parse packet header ───────────────────────────────────────────
    CryptoFrame frame;
    uint8_t plainBuf[PAYLOAD_SIZE];
    uint8_t cipherBuf[PAYLOAD_SIZE + 16];

    frame.mode = LoRa.read();
    LoRa.readBytes(frame.iv,  16);
    LoRa.readBytes(frame.tag, 16);

    uint32_t rxSeq = 0;
    LoRa.readBytes((uint8_t*)&rxSeq, 4);

    frame.cipher = cipherBuf;
    frame.len    = pktSize - HEADER_SIZE;
    if (frame.len > sizeof(cipherBuf)) return;   // sanity guard
    LoRa.readBytes(cipherBuf, frame.len);

    // ── 2. Track lost packets via sequence gaps ───────────────────────────
    // Handle TX restarts (sequence wraps back to small values) so PER does
    // not get corrupted by unsigned underflow artifacts.
    uint32_t prevSeq = stats.lastSeq;
    if (stats.lastSeq == 0) {
        stats.lastSeq = rxSeq;
    } else if (rxSeq > stats.lastSeq) {
        uint32_t gap = rxSeq - stats.lastSeq - 1;
        stats.lost += gap;
        stats.lastSeq = rxSeq;
    } else if (rxSeq < stats.lastSeq && rxSeq < 100 && stats.lastSeq > 1000) {
        stats.total = 0;
        stats.lost = 0;
        stats.authFails = 0;
        stats.lastSeq = rxSeq;
        Serial.printf("[RX] Sequence restart detected (last=%lu, now=%lu), resetting link counters\n",
                      (unsigned long)prevSeq,
                      (unsigned long)rxSeq);
    }

    // ── 3. Decrypt ───────────────────────────────────────────────────────
    bool authOk = sec.decrypt(frame, plainBuf);

    // Accumulate in a very simple window (full sliding window is optional here;
    // keeping it simple: running totals, ACK builder divides by total)
    if (!authOk && frame.mode == (uint8_t)AESMode::GCM)
        stats.authFails++;
    stats.total++;

    // ── 4. Process plaintext if decryption succeeded ─────────────────────
    if (authOk) {
        const Telemetry* t = reinterpret_cast<const Telemetry*>(plainBuf);
        Serial.printf("[RX] #%u mode=%u T=%.2f P=%.2f lat=%.4f lng=%.4f\n",
                      t->seqNum,
                      frame.mode,
                      t->temperature,
                      t->pressure,
                      t->latitude,
                      t->longitude);
    } else {
        Serial.printf("[RX] #%u AUTH FAIL (mode=%u)\n", rxSeq, frame.mode);
    }

    // ── 5. Build and send ACK with link metrics ───────────────────────────
    // Give the radio a short turnaround gap (avoids half-duplex collision)
    delay(20);

    int8_t snr = (int8_t)LoRa.packetSnr();

    ACKMetrics ack = sec.buildACK(snr,
                                   stats.total,
                                   stats.lost,
                                   stats.authFails);

    const float perRadio = (stats.total > 0) ? (float)stats.lost / (float)stats.total : 0.0f;
    const float perAuth  = (stats.total > 0) ? (float)stats.authFails / (float)stats.total : 0.0f;
    const float battEst  = 1.0f;
    const SecurityPolicy dbgPolicy = COST_DEBUG_POLICY;

    const float cCTR = computeDebugCost(AESMode::CTR, dbgPolicy, (float)snr, perRadio, perAuth, battEst);
    const float cCBC = computeDebugCost(AESMode::CBC, dbgPolicy, (float)snr, perRadio, perAuth, battEst);
    const float cGCM = computeDebugCost(AESMode::GCM, dbgPolicy, (float)snr, perRadio, perAuth, battEst);
    const float cRxMode = computeDebugCost(static_cast<AESMode>(frame.mode),
                                           dbgPolicy, (float)snr, perRadio, perAuth, battEst);

    LoRa.beginPacket();
    LoRa.write((const uint8_t*)&ack, sizeof(ack));
    LoRa.endPacket();

    Serial.printf("[RX] ACK sent snr=%d per=%.3f af=%.3f\n",
                  ack.snrRaw,
                  ack.perQ8    / 256.0f,
                  ack.authFailQ8 / 256.0f);
    Serial.printf("[RX] COST policy=BestEffort snr=%.1f perR=%.3f perA=%.3f batt=%.2f mode=%s cost=%.4f (CTR=%.4f CBC=%.4f GCM=%.4f)\n",
                  (float)snr,
                  perRadio,
                  perAuth,
                  battEst,
                  modeName(static_cast<AESMode>(frame.mode)),
                  cRxMode,
                  cCTR,
                  cCBC,
                  cGCM);
}
