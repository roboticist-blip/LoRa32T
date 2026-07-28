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

// LoRa pin mapping (adjust to your board)
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
// Header size is mode-dependent now (M-1) — see sec.frameHeaderSize(mode).
// The minimum possible header (CTR/GCM nonce, no tag) is used only for the
// initial parsePacket() length sanity check; the real header length is
// resolved once the mode byte is read.
#define MIN_HEADER_SIZE (1 + 12 + 4)

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
    uint32_t received;    // packets actually received (was "total")
    uint32_t lost;
    uint32_t authFails;   // GCM-only tag mismatches
    uint32_t crcFails;    // cross-mode corruption proxy (M-3)
    uint32_t lastSeq;
    uint8_t  windowLost[WINDOW_SIZE];
    uint8_t  windowAuth[WINDOW_SIZE];
    uint8_t  head;
};

// M-4: PER's correct denominator is everything that was SENT
// (received + lost), not just what arrived.
inline uint32_t expectedTotal(const RxStats& s) { return s.received + s.lost; }

LoRa32T sec;
RxStats         stats = {};
bool ledPulseActive = false;
uint32_t ledOffAtMs = 0;

static constexpr float COST_ALPHA  = 0.55f;
static constexpr float COST_BETA   = 0.30f;
static constexpr float COST_GAMMA0 = 0.10f;
static constexpr float COST_KAPPA  = 0.25f;
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

// Debug-only mirror of the TX-side cost function, for the log line below.
// Uses sec.toaMs()/sec.frameHeaderSize() so it stays consistent with the
// library's live ToA model (M-1) instead of its own hardcoded constants.
// This is diagnostic output only — the RX never makes mode decisions.
float computeDebugCost(AESMode mode,
                       SecurityPolicy policy,
                       float snr,
                       float perRadio,
                       float perAuth,
                       float battLevel)
{
    if (policy == SecurityPolicy::Mandatory && mode != AESMode::GCM)
        return 1e9f;
    // M-6: boundary matches the library (>= 0 dB, not > 0 dB).
    if (policy == SecurityPolicy::Conditional && mode == AESMode::CTR && snr >= 0.0f)
        return 1e9f;

    float perEff = perRadio;
    if (mode == AESMode::GCM)
        perEff += perAuth;

    float toaCBC = sec.toaMs(AESMode::CBC, PAYLOAD_SIZE);
    float toaM   = sec.toaMs(mode,         PAYLOAD_SIZE);
    float toaNorm = (toaCBC > 0.0f) ? (toaM / toaCBC) : 1.0f;

    const float fLink    = perEff;
    const float fLatency = toaNorm * retryFactor(perEff);
    const float fEnergy  = toaNorm;
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
    if (pktSize < (int)(MIN_HEADER_SIZE + PAYLOAD_SIZE)) return;

    digitalWrite(LED_PIN, HIGH);
    ledPulseActive = true;
    ledOffAtMs = millis() + LED_BLINK_MS;

    // 1. Parse mode byte first — header length depends on it (M-1)
    CryptoFrame frame;
    uint8_t plainBuf[PAYLOAD_SIZE];
    uint8_t cipherBuf[PAYLOAD_SIZE + 16];
    memset(frame.iv,  0, 16);
    memset(frame.tag, 0, 16);

    frame.mode = LoRa.read();
    AESMode frameMode = static_cast<AESMode>(frame.mode);

    // 2. Enforce SecurityPolicy on the RECEIVE path before decrypting
    //      (S-2). Previously Mandatory/Conditional were only checked at
    //      the transmitter's mode-selection step, which an attacker does
    //      not need to touch to send a frame in a disallowed mode.
    if (!sec.isModeAllowed(frameMode, COST_DEBUG_POLICY)) {
        Serial.printf("[RX] Rejected frame: mode=%u not permitted by policy\n", frame.mode);
        stats.crcFails++;
        return;
    }

    size_t ivLen = (frameMode == AESMode::CBC) ? 16 : 12;
    LoRa.readBytes(frame.iv, ivLen);
    if (frameMode == AESMode::GCM)
        LoRa.readBytes(frame.tag, 16);

    uint32_t rxSeq = 0;
    LoRa.readBytes((uint8_t*)&rxSeq, 4);

    size_t headerSize = 1 + ivLen + (frameMode == AESMode::GCM ? 16 : 0) + 4;
    frame.cipher = cipherBuf;
    frame.len    = pktSize - headerSize;
    if (frame.len > sizeof(cipherBuf) || (int)frame.len < 0) return;   // sanity guard
    LoRa.readBytes(cipherBuf, frame.len);

    // 3. Track lost packets via sequence gaps─
    uint32_t prevSeq = stats.lastSeq;
    if (stats.lastSeq == 0) {
        stats.lastSeq = rxSeq;
    } else if (rxSeq > stats.lastSeq) {
        uint32_t gap = rxSeq - stats.lastSeq - 1;
        stats.lost += gap;
        stats.lastSeq = rxSeq;
    } else if (rxSeq < stats.lastSeq && rxSeq < 100 && stats.lastSeq > 1000) {
        stats.received  = 0;
        stats.lost      = 0;
        stats.authFails = 0;
        stats.crcFails  = 0;
        stats.lastSeq   = rxSeq;
        Serial.printf("[RX] Sequence restart detected (last=%lu, now=%lu), resetting link counters\n",
                      (unsigned long)prevSeq,
                      (unsigned long)rxSeq);
    }

    // 4. Decryp
    bool authOk = sec.decrypt(frame, plainBuf);
    stats.received++;

    if (!authOk) {
        // GCM-specific tag mismatch, kept for its own reported metric...
        if (frameMode == AESMode::GCM)
            stats.authFails++;
        // ...but ALSO folded into the cross-mode corruption proxy (M-3),
        // so perAuth stays a live signal even while parked in CTR/CBC,
        // where GCM's own counter would otherwise be permanently zero.
        stats.crcFails++;
        sec.noteCrcResult(false);
    } else {
        sec.noteCrcResult(true);
    }

    // 5. Process plaintext if decryption succeede
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
        Serial.printf("[RX] #%u AUTH/DECODE FAIL (mode=%u)\n", rxSeq, frame.mode);
    }

    // 6. Build and send authenticated ACK with link metrics─
    delay(20);

    int8_t snr = (int8_t)LoRa.packetSnr();

    // M-4: PER denominator is total SENT (received + lost), not just
    // what arrived. The previous lost/received formulation overstated
    // PER without bound as the link degraded (e.g. reports 1.0 at true
    // 50% loss instead of 0.5).
    uint32_t denom = expectedTotal(stats);
    ACKMetrics ack = sec.buildACK(snr, denom, stats.lost, stats.crcFails);

    const float perRadio = (denom > 0) ? (float)stats.lost     / (float)denom : 0.0f;
    const float perAuth  = (denom > 0) ? (float)stats.crcFails / (float)denom : 0.0f;
    const float battEst  = 1.0f;  // RX is mains-powered; not part of the cost model
    const SecurityPolicy dbgPolicy = COST_DEBUG_POLICY;

    const float cCTR = computeDebugCost(AESMode::CTR, dbgPolicy, (float)snr, perRadio, perAuth, battEst);
    const float cCBC = computeDebugCost(AESMode::CBC, dbgPolicy, (float)snr, perRadio, perAuth, battEst);
    const float cGCM = computeDebugCost(AESMode::GCM, dbgPolicy, (float)snr, perRadio, perAuth, battEst);
    const float cRxMode = computeDebugCost(frameMode, dbgPolicy, (float)snr, perRadio, perAuth, battEst);

    LoRa.beginPacket();
    LoRa.write((const uint8_t*)&ack, sizeof(ack));
    LoRa.endPacket();

    Serial.printf("[RX] ACK sent snr=%d per=%.3f af=%.3f ctr=%lu\n",
                  ack.snrRaw,
                  ack.perQ8    / 255.0f,
                  ack.authFailQ8 / 255.0f,
                  (unsigned long)ack.counter);
    Serial.printf("[RX] COST policy=BestEffort snr=%.1f perR=%.3f perA=%.3f batt=%.2f mode=%s cost=%.4f (CTR=%.4f CBC=%.4f GCM=%.4f)\n",
                  (float)snr,
                  perRadio,
                  perAuth,
                  battEst,
                  modeName(frameMode),
                  cRxMode,
                  cCTR,
                  cCBC,
                  cGCM);
}
