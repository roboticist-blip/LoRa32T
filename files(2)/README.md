# LoRa32T

Deterministic cross-layer adaptive AES mode selection for **ESP32 + SX1278 LoRa** systems.

Dynamically switches between **AES-GCM**, **AES-CBC**, and **AES-CTR** based on live channel conditions — without any machine learning. Based on the paper:

> *"Cross-Layer Adaptive AES Mode Selection for ESP32-Based LoRa Telemetry Systems: A Deterministic Framework Under Link Variability"* — Maheshwari et al., 2026

## How it works

The transmitter runs a cost function over three competing objectives:

| Term | Weight | Controls |
|------|--------|----------|
| Reliability (`α`) | 0.55 | Penalises packet loss + GCM auth failures |
| Latency (`β`) | 0.30 | Penalises high Time-on-Air |
| Energy (`γ`) | 0.10–0.35 | Scales up automatically as battery drains |

The receiver measures SNR, PER, and GCM authentication failure rate, packs them into a 3-byte ACK, and sends it back. The transmitter makes all mode decisions.

## Installation

**Arduino Library Manager:** search for `LoRa32T`

**PlatformIO:** add to `platformio.ini`:
```ini
lib_deps = YOUR_USERNAME/LoRa32T
```

**Manual:** clone into your `libraries/` folder:
```bash
git clone https://github.com/YOUR_USERNAME/LoRa32T
```

## Dependencies

- [sandeepmistry/LoRa](https://github.com/sandeepmistry/arduino-LoRa) `^0.8.0`
- ESP32 Arduino core `>= 2.0.0` (provides `mbedtls`)

## Quick Start

**Transmitter:**
```cpp
#include "LoRa32T.h"

LoRa32T sec;
const uint8_t PSK[16] = { /* your 16-byte key */ };

void setup() {
    sec.begin(PSK);
}

void loop() {
    sec.updateLinkState(snr, perRadio, perAuth, battLevel);
    AESMode mode = sec.selectMode(SecurityPolicy::Conditional);

    CryptoFrame frame;
    frame.cipher = cipherBuf;
    sec.encrypt(plaintext, 64, seqNum, frame);
    // transmit frame ...
}
```

**Receiver:**
```cpp
void loop() {
    // receive packet, then:
    bool ok = sec.decrypt(frame, plainBuf);
    ACKMetrics ack = sec.buildACK(rawSnr, total, lost, authFails);
    // send ack back to TX ...
}
```

See the `examples/` folder for complete TX and RX sketches.

## Security Policies

| Policy | Behaviour |
|--------|-----------|
| `BestEffort` | Full cost-driven selection, all modes allowed |
| `Conditional` | GCM preferred; CTR only when SNR < 0 dB |
| `Mandatory` | GCM always (e.g. medical / regulated data) |

## Tuning

```cpp
CostWeights w = {
    .alpha  = 0.55f,   // reliability priority
    .beta   = 0.30f,   // latency priority
    .gamma0 = 0.10f,   // base energy weight
    .kappa  = 0.25f    // battery sensitivity
};
StabilityConfig s = {
    .hystThresh      = 0.05f,  // 5% cost margin required to switch
    .minDwellPackets = 20      // packets before reconsidering mode
};
sec.begin(PSK, w, s);
```

## Hardware

- **MCU:** ESP32 (Xtensa LX6, hardware AES acceleration via mbedtls)
- **Radio:** Semtech SX1278, 433 MHz ISM band
- **Settings used in paper:** SF7, BW 125 kHz, CR 4/5, 14 dBm

## License

MIT
