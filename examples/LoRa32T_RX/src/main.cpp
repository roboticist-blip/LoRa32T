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
    uint32_t gap = (stats.lastSeq > 0) ? (rxSeq - stats.lastSeq - 1) : 0;
    stats.lost += gap;
    stats.lastSeq = rxSeq;

    // ── 3. Decrypt ───────────────────────────────────────────────────────
    bool authOk = sec.decrypt(frame, plainBuf);

    uint8_t authFailFlag = (!authOk && frame.mode == (uint8_t)AESMode::GCM) ? 1 : 0;

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

    LoRa.beginPacket();
    LoRa.write((const uint8_t*)&ack, sizeof(ack));
    LoRa.endPacket();

    Serial.printf("[RX] ACK sent snr=%d per=%.3f af=%.3f\n",
                  ack.snrRaw,
                  ack.perQ8    / 256.0f,
                  ack.authFailQ8 / 256.0f);
}
