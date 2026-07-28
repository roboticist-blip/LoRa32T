/*
 * Transmitter side – ESP32 + SX1278 LoRa
 *
 * Periodically encrypts a telemetry payload, transmits it, waits for the
 * ACK that carries link metrics from the receiver, then lets the library
 * pick the best AES mode for the next packet.
 *
 * Wiring (VSPI):
 *   SX1278 NSS  → GPIO 5
 *   SX1278 MOSI → GPIO 23
 *   SX1278 MISO → GPIO 19
 *   SX1278 SCK  → GPIO 18
 *   SX1278 RST  → GPIO 14
 *   SX1278 DIO0 → GPIO 26
 */

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include "LoRa32T.h"

//LoRa pin mapping 
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 26
#define LED_PIN   13
static const uint32_t LED_BLINK_MS = 80;

#define RXD2 33
#define TXD2 32

#define GPS_BAUD 9600
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

//Pre-shared AES-128 key (must match RX) 
static const uint8_t PSK[16] = {
    0xDE,0xAD,0xBE,0xEF, 0xCA,0xFE,0xBA,0xBE,
    0x01,0x23,0x45,0x67, 0x89,0xAB,0xCD,0xEF
};

// Packet interval and ACK timeout 
#define TX_INTERVAL_MS   1000   // 1 Hz telemetry
#define ACK_TIMEOUT_MS    600   // RX must reply within 600 ms

// Wire protocol: mode-dependent packet layout (M-1) 
// [ 1 mode | iv/nonce (16 for CBC, 12 for CTR/GCM) | tag (16, GCM only) | 4 seqNum | N ciphertext ]
// Header size is no longer a compile-time constant: sec.frameHeaderSize(mode)
// gives the real value, and that's what must go on the wire so airtime
// actually differs between modes the way the cost model assumes.
#define PAYLOAD_SIZE 64                   // fixed telemetry payload

// Telemetry data structure (64 bytes exactly)
struct Telemetry {
    uint32_t seqNum;
    float    temperature;
    float    pressure;
    float    latitude;
    float    longitude;
    uint32_t timestamp;
    uint8_t  pad[64 - 4 - 4*4 - 4];  // fills to exactly 64 bytes
};
static_assert(sizeof(Telemetry) == PAYLOAD_SIZE, "Telemetry must be 64 bytes");

// Globals
LoRa32T sec;
uint32_t        seqNum   = 0;
uint32_t        lastTxMs = 0;
bool            ledPulseActive = false;
uint32_t        ledOffAtMs = 0;
uint32_t        sentPackets = 0;
uint32_t        ackPackets = 0;
bool            rxReachable = false;

void serviceGPS()
{
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
}

// Battery simulation: read ADC pin connected to voltage divider on VBAT
// Adjust the pin and divider ratio for your hardware.
#define BATT_ADC_PIN 15
float readBatteryLevel()
{
    uint16_t raw  = analogRead(BATT_ADC_PIN);
    float vAdc    = raw * (3.3f / 4095.0f);
    float vBat    = vAdc * 2.0f;
    float level   = (vBat - 3.0f) / (4.2f - 3.0f);
    return constrain(level, 0.0f, 1.0f);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Serial 2 started at 9600 baud rate");
     
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6)) {
        Serial.println("[TX] LoRa init failed");
        while (true);
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(14);
    LoRa.enableCrc();

    // Initialise library with default weights and stability config
    sec.begin(PSK);

    Serial.println("[TX] Ready");
}

void loop()
{
    serviceGPS();

    if (ledPulseActive && (int32_t)(millis() - ledOffAtMs) >= 0) {
        digitalWrite(LED_PIN, LOW);
        ledPulseActive = false;
    }

    if (millis() - lastTxMs < TX_INTERVAL_MS) return;
    lastTxMs = millis();

//1. Build telemetry
Telemetry telem = {};
telem.seqNum = ++seqNum;

telem.temperature = 25.3f + (float)(esp_random() % 100) * 0.01f;

// Only update if GPS has a valid fix
if (gps.location.isValid()) {
    telem.latitude  = gps.location.lat();
    telem.longitude = gps.location.lng();
} else {
    telem.latitude  = 0.0f;
    telem.longitude = 0.0f;
}
telem.timestamp = (uint32_t)(millis() / 1000);

    //2. Update battery level (direct reading, not link-EMA'd)
    sec.setBatteryLevel(readBatteryLevel());

    // Occasionally force a GCM probe even while parked in CTR/CBC, so the
    // controller has a live authentication-failure sample to decide
    // whether it's safe to return to GCM (M-3). Mandatory/Conditional
    // policy still governs whether the probe is actually permitted.
    SecurityPolicy activePolicy = SecurityPolicy::BestEffort;
    bool probing = sec.shouldProbeGCM();

    AESMode mode = sec.selectMode(activePolicy);
    if (probing && mode != AESMode::GCM && sec.isModeAllowed(AESMode::GCM, activePolicy)) {
        mode = AESMode::GCM;
    }

    // 3. Encrypt
    uint8_t     cipherBuf[PAYLOAD_SIZE + 16];  // +16 headroom for CBC padding
    CryptoFrame frame;
    frame.cipher = cipherBuf;
    frame.mode   = static_cast<uint8_t>(mode);

    bool ok = sec.encrypt((const uint8_t*)&telem, sizeof(telem), seqNum, frame);
    if (!ok) {
        Serial.println("[TX] Encrypt failed");
        return;
    }

    // 4. Transmit packet
    // Only the bytes this mode actually uses go on the wire: no more
    // padding 16 zero tag bytes into CTR/CBC. This is what makes airtime
    // (and therefore the latency/energy terms of the cost function) an
    // actual measured difference between modes instead of an assumption.
    size_t ivLen = (mode == AESMode::CBC) ? 16 : 12;
    LoRa.beginPacket();
    LoRa.write(frame.mode);
    LoRa.write(frame.iv, ivLen);
    if (mode == AESMode::GCM)
        LoRa.write(frame.tag, 16);
    LoRa.write((uint8_t*)&seqNum, 4);
    LoRa.write(frame.cipher, frame.len);
    LoRa.endPacket();
    sentPackets++;

    digitalWrite(LED_PIN, HIGH);
    ledPulseActive = true;
    ledOffAtMs = millis() + LED_BLINK_MS;

    Serial.printf("[TX] #%u mode=%s bat=%.2f snr=%.1f\n",
                  seqNum,
                  mode == AESMode::GCM ? "GCM" :
                  mode == AESMode::CBC ? "CBC" : "CTR",
                  readBatteryLevel(),
                  sec.linkState().snr);

    // 5. Wait for ACK with link metrics
    uint32_t deadline = millis() + ACK_TIMEOUT_MS;
    bool ackReceived = false;
    while (millis() < deadline) {
        serviceGPS();

        int pktSize = LoRa.parsePacket();
        if (pktSize == (int)sizeof(ACKMetrics)) {
            ACKMetrics ack;
            LoRa.readBytes((uint8_t*)&ack, sizeof(ack));

            // Reject forged or replayed ACKs BEFORE they touch link state
            // (S-4). Without this, a single crafted 3-byte-equivalent
            // packet with authFailQ8=255 could force a downgrade out of
            // GCM; verifyACK() checks both the HMAC and a monotonic
            // counter.
            if (!sec.verifyACK(ack)) {
                Serial.println("[TX] Rejected ACK: bad MAC or replay");
                delay(5);
                continue;
            }

            sec.updateLinkMetrics(ack);
            ackReceived = true;
            ackPackets++;
            rxReachable = true;
            Serial.printf("[TX] ACK snr=%d per=%.2f af=%.2f switches=%u\n",
                          ack.snrRaw,
                          ack.perQ8    / 255.0f,
                          ack.authFailQ8 / 255.0f,
                          sec.modeSwitchCount());
            break;
        }
        delay(5);
    }

    if (!ackReceived) {
        rxReachable = false;
        Serial.printf("[TX] No ACK for #%u -> RX unreachable (from TX view)\n", seqNum);
    }

    Serial.printf("[TX] LinkCheck sent=%u ack=%u reach=%.2f status=%s\n",
                  sentPackets,
                  ackPackets,
                  sentPackets ? (100.0f * (float)ackPackets / (float)sentPackets) : 0.0f,
                  rxReachable ? "RX_OK" : "RX_MISS");
}
