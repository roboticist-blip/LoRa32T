# LoRa32T

**Deterministic, cross-layer adaptive AES mode selection for ESP32 + SX1278 LoRa telemetry.**

LoRa32T dynamically selects between **AES-GCM**, **AES-CBC**, and **AES-CTR** on a
packet-by-packet basis, driven by measured link quality, GCM authentication
health, and transmitter battery level. Mode selection is a transparent,
weighted cost function evaluated on the transmitter — no machine learning,
no black-box behavior, and every decision is traceable to the metrics that
produced it.

Companion paper: *"Cross-Layer Adaptive AES Mode Selection for LoRa-Based
Telemetry Systems: A Deterministic Framework Under Link Variability."*

## Features

- **Three AES-128 modes, one link-aware controller.** GCM for authenticated
  encryption when the channel supports it, CTR/CBC as lower-overhead
  fallbacks when it doesn't.
- **Transmitter-owned decisions.** The receiver only measures and reports;
  all mode selection and policy enforcement happens at the transmitter.
- **Authenticated ACK feedback.** Link metrics (SNR, PER, GCM auth-failure
  rate) travel back from receiver to transmitter in a compact, HMAC-tagged,
  replay-protected ACK.
- **Stable by design.** Relative-cost hysteresis and a minimum dwell time
  prevent mode oscillation under noisy or fast-varying channels.
- **Policy-gated.** Three enforceable security policies (`BestEffort`,
  `Conditional`, `Mandatory`) let deployments require authenticated
  encryption regardless of what the cost function alone would pick.
- **Real airtime accounting.** Time-on-air is computed per mode from the
  actual wire frame size (nonce/IV/tag overhead, CBC block padding), not a
  fixed assumption — so the latency and energy terms reflect what's really
  transmitted.

## How It Works

Every packet cycle, the transmitter evaluates a cost for each of the three
modes and selects the minimum:

```
C(mode) = α · Flink + β · Flatency + γ(battery) · Fenergy + δ · Fsecurity + ε · Freuse
```

| Term | Captures |
|---|---|
| `Flink` | Packet loss, plus GCM authentication failures when the mode is GCM |
| `Flatency` | Normalized time-on-air, scaled by expected retransmissions |
| `Fenergy` | Normalized time-on-air (airtime dominates LoRa energy use) |
| `Fsecurity` | Credit for GCM packets that authenticate successfully |
| `Freuse` | Risk of CTR nonce reuse on retransmission, penalizing CTR under high retry load |

`γ` grows automatically as the battery drains, shifting priority toward
energy efficiency. A relative-hysteresis threshold and a minimum dwell time
(both configurable) gate every mode switch, so a mode change only happens
when it's worth the switching cost and the link has actually settled.

The receiver calculates SNR, packet error rate, and GCM authentication failure counts, which are sent back in an ACK that includes an authenticated HMAC-SHA256-truncation tag along with a monotonic counter to protect against forgeries or replays of measurements (see Security Properties).

## Installation

**Arduino Library Manager:** search for `LoRa32T`.

**PlatformIO**, add to `platformio.ini`:
```ini
lib_deps = roboticist-blip/LoRa32T
```

**Manual**, clone into your Arduino `libraries/` folder:
```bash
git clone https://github.com/roboticist-blip/LoRa32T.git
```

### Dependencies

- [sandeepmistry/LoRa](https://github.com/sandeepmistry/arduino-LoRa) `^0.8.0`
- ESP32 Arduino core `>= 2.0.0` (provides the `mbedtls` AES/GCM backend)

## Quick Start

**Transmitter**
```cpp
#include "LoRa32T.h"

LoRa32T sec;
const uint8_t PSK[16] = { /* pre-shared 16-byte AES-128 key */ };

void setup() {
    sec.begin(PSK);
}

void loop() {
    sec.setBatteryLevel(readBatteryLevel());

    bool probing = sec.shouldProbeGCM();
    AESMode mode = sec.selectMode(SecurityPolicy::BestEffort);
    if (probing && mode != AESMode::GCM &&
        sec.isModeAllowed(AESMode::GCM, SecurityPolicy::BestEffort)) {
        mode = AESMode::GCM;
    }

    CryptoFrame frame;
    frame.cipher = cipherBuf;
    frame.mode   = static_cast<uint8_t>(mode);
    sec.encrypt(plaintext, sizeof(plaintext), seqNum, frame);
    // transmit frame.mode, frame.iv, frame.tag (GCM only), seqNum, frame.cipher ...

    // Wait for the ACK, reject anything that doesn't verify, then apply it:
    if (sec.verifyACK(ack)) {
        sec.updateLinkMetrics(ack);
    }
}
```

**Receiver**
```cpp
void loop() {
    // Parse mode + IV/nonce + tag + seqNum + ciphertext from the incoming packet.

    if (!sec.isModeAllowed(frameMode, activePolicy)) {
        // Reject: frame's mode violates the receiver's own policy.
        return;
    }

    bool ok = sec.decrypt(frame, plainBuf);
    sec.noteCrcResult(ok);   // keeps perAuth informative even while in CTR/CBC

    ACKMetrics ack = sec.buildACK(rawSnr, totalReceived, lostCount, authFailCount);
    // send ack back to the transmitter ...
}
```

See `examples/LoRa32T_TX` and `examples/LoRa32T_RX` for complete, wiring-annotated sketches.

## Security Policies

| Policy | Behavior |
|---|---|
| `BestEffort` | Full cost-driven selection; all three modes eligible |
| `Conditional` | GCM preferred; CBC always eligible as fallback; CTR only permitted once SNR ≥ 0 dB |
| `Mandatory` | GCM required at all times, regardless of cost or channel |

Policy is enforced on **both** ends: the transmitter won't select a
disallowed mode, and the receiver independently checks the frame's mode
byte against its own policy via `isModeAllowed()` before attempting
decryption — a receiver never trusts the transmitter's mode choice
implicitly.

## Tuning

```cpp
CostWeights w = {
    .alpha   = 0.55f,  // reliability priority
    .beta    = 0.30f,  // latency priority
    .gamma0  = 0.10f,  // base energy weight
    .kappa   = 0.25f,  // battery sensitivity
    .delta   = 0.15f,  // GCM security-utility credit
    .epsilon = 0.20f   // CTR nonce-reuse-risk weight
};
StabilityConfig s = {
    .hystThresh      = 0.05f,  // relative cost margin required to switch
    .minDwellPackets = 20,     // packets before a mode change is reconsidered
    .emaAlpha        = 0.3f    // link-metric smoothing coefficient
};
sec.begin(PSK, w, s);
```

## Wire Format

Frame layout is mode-dependent; header size is computed at runtime via
`frameHeaderSize()`, not assumed as a fixed constant:

```
[ 1B mode | IV/nonce (16B CBC, 12B CTR/GCM) | tag (16B, GCM only) | 4B seqNum | ciphertext ]
```

CBC ciphertext is PKCS#7-padded to a 16-byte boundary; CTR and GCM are
stream ciphers and carry no padding. `toaMs()` accounts for both the frame
header and this padding when computing per-mode airtime.

## Security Properties

- **Authenticated ACKs.** Every ACK carries a truncated HMAC-SHA256 tag
  under the same pre-shared key, plus a monotonic counter — a forged or
  replayed ACK cannot be used to force a downgrade.
- **Mode-bound AAD.** The frame's mode byte is included in GCM's
  associated data, so a GCM receiver cannot be handed a relabeled CTR/CBC
  frame, and vice versa.
- **Receiver-side policy enforcement.** `isModeAllowed()` is checked before
  `decrypt()` is ever called, independent of what the transmitter sent.
- **Cross-mode integrity signal.** `noteCrcResult()` keeps the controller's
  authentication-failure estimate informative even while parked in a
  non-authenticated mode, via periodic GCM probing.

### Non-Goals

- No key exchange or rotation — keys are pre-shared and provisioned out of band.
- No side-channel (DPA/EM) hardening beyond what the ESP32 AES peripheral provides natively.
- No multi-node collision handling — designed for a single TX/RX pair.

## Hardware

- **MCU:** ESP32 (Xtensa LX6, hardware AES acceleration via `mbedtls`)
- **Radio:** Semtech SX1278, 433 MHz ISM band
- **Reference settings:** SF7, BW 125 kHz, CR 4/5, 14 dBm, explicit header

## Repository Layout

```
src/                  Library core (LoRa32T.h / LoRa32T.cpp)
examples/LoRa32T_TX/  Transmitter sketch, GPS + battery telemetry
examples/LoRa32T_RX/  Receiver sketch, ACK generation
examples/*.ino        Minimal single-file TX/RX examples
Data/                 Captured and simulated link logs used in evaluation
```

## License

MIT — see [LICENSE](LICENSE).
