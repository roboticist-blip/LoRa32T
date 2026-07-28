#pragma once

#include <Arduino.h>
#include <mbedtls/aes.h>
#include <mbedtls/gcm.h>
#include <mbedtls/md.h>

enum class AESMode : uint8_t {
    CTR = 0,
    CBC = 1,
    GCM = 2
};

enum class SecurityPolicy : uint8_t {
    BestEffort  = 0,  // Cost-driven, all modes permitted
    Conditional = 1,  // GCM preferred; CTR only when SNR >= 0 dB
    Mandatory   = 2   // GCM always, regardless of channel
};

struct LinkState {
    float    snr;
    float    perRadio;
    float    perAuth;
    float    battLevel;
    SecurityPolicy policy;
};

// ACKs are authenticated: a monotonic counter (replay defence) plus an
// 8-byte truncated HMAC-SHA256 tag over {snrRaw, perQ8, authFailQ8, counter}
// computed under the same pre-shared key used for telemetry. See S-4.
struct ACKMetrics {
    int8_t   snrRaw;
    uint8_t  perQ8;
    uint8_t  authFailQ8;
    uint32_t counter;
    uint8_t  mac[8];
};

// Wire-level frame. `iv` holds either a 16-byte random IV (CBC) or a
// 12-byte nonce zero-padded to 16 (CTR/GCM); `tag` is only meaningful,
// and only transmitted, when mode == GCM (see M-1 / frameHeaderSize()).
struct CryptoFrame {
    uint8_t  mode;
    uint8_t  iv[16];
    uint8_t  tag[16];
    uint8_t* cipher;
    size_t   len;
};

struct CostWeights {
    float alpha;   // Reliability weight  (default 0.55)
    float beta;    // Latency weight      (default 0.30)
    float gamma0;  // Base energy weight  (default 0.10)
    float kappa;   // Battery sensitivity (default 0.25)
};

struct StabilityConfig {
    float    hystThresh;      // RELATIVE cost margin required to switch (e.g. 0.05 = 5%)
    uint16_t minDwellPackets;
    float    emaAlpha;        // Link-metric EMA smoothing coefficient (default 0.3)
};

// LoRa PHY parameters, needed to turn frame size into real airtime (M-1).
struct RadioConfig {
    uint8_t  sf;        // Spreading factor (7-12)
    float    bwHz;       // Bandwidth in Hz (e.g. 125000)
    uint8_t  cr;         // Coding rate denominator+4, i.e. 4/5 -> 5
    bool     explicitHeader;
    bool     lowDataRateOptimize;
    uint8_t  preambleSymbols;
};


class LoRa32T {
public:
    LoRa32T();
    void begin(const uint8_t key[16],
               CostWeights    weights  = {0.55f, 0.30f, 0.10f, 0.25f},
               StabilityConfig stab    = {0.05f, 20, 0.3f},
               RadioConfig     radio   = {7, 125000.0f, 5, true, false, 8});

    // Link-state updates. NOTE: updateLinkMetrics() is the only path that
    // should feed values derived from the ACK — it applies the EMA once.
    // Do not re-smooth an already-smoothed value (see M-7); use
    // setBatteryLevel() for the transmitter's own battery reading, which
    // is a direct measurement and must not be filtered through the link EMA.
    void updateLinkMetrics(const ACKMetrics& ack);
    void setBatteryLevel(float battLevel);

    // Cross-mode failure signal (M-3): call this once per received packet
    // regardless of AES mode, using the radio's own CRC result (or, for
    // GCM, the GMAC result) as a generic "packet was corrupted" proxy.
    // This keeps perAuth informative even while the controller is
    // operating in CTR/CBC, where GCM's own auth counter is silent.
    void noteCrcResult(bool crcOk);

    // Returns true if a GCM probe packet should be sent this cycle, based
    // on a fixed probe interval (default: every minDwellPackets packets).
    // Sending an occasional GCM packet while parked in CTR/CBC is what
    // lets the controller detect that conditions have improved enough to
    // return to authenticated encryption (M-3).
    bool shouldProbeGCM();

    AESMode selectMode(SecurityPolicy policy = SecurityPolicy::BestEffort);

    // isModeAllowed() must be checked on the RECEIVE path before decrypt()
    // is even attempted, so that SecurityPolicy is enforced against the
    // frame's mode byte rather than trusted implicitly (S-2). The frame's
    // mode byte itself is also bound into the GCM AAD, so a receiver
    // running GCM cannot be fed a CTR/CBC frame silently relabelled GCM,
    // and vice versa.
    bool isModeAllowed(AESMode frameMode, SecurityPolicy policy) const;

    bool encrypt(const uint8_t* plain, size_t plainLen,
                 uint32_t seqNum, CryptoFrame& frame);
    bool decrypt(const CryptoFrame& frame, uint8_t* plain);

    // Build an authenticated ACK. Increments and embeds the local ACK
    // counter, then HMACs the metrics so a forged/replayed ACK (S-4)
    // cannot be used to force a mode downgrade.
    ACKMetrics buildACK(int8_t rawSnr,
                        uint32_t rxTotal, uint32_t rxLost,
                        uint32_t authFails);

    // Verifies an inbound ACK's MAC and rejects stale/replayed counters.
    // Call this before updateLinkMetrics(). Returns false on bad MAC or
    // non-increasing counter, in which case the ACK MUST be discarded.
    bool verifyACK(const ACKMetrics& ack);

    // Real per-mode frame size (mode byte + iv/nonce + tag-if-present + seq),
    // and the resulting airtime for a given payload, computed from the
    // Semtech LoRa ToA formula rather than a hardcoded constant (M-1).
    size_t frameHeaderSize(AESMode m) const;
    float  toaMs(AESMode m, size_t payloadLen) const;

    const LinkState& linkState() const { return _ls; }
    AESMode currentMode() const { return _mode; }
    uint32_t modeSwitchCount() const { return _switchCount; }
    float lastCost(AESMode m) const { return _lastCost[static_cast<uint8_t>(m)]; }

private:
    uint8_t         _key[16];
    CostWeights     _w;
    StabilityConfig _stab;
    RadioConfig     _radio;
    LinkState       _ls;
    AESMode         _mode;
    uint16_t        _dwellCount;
    uint32_t        _switchCount;
    uint32_t        _packetsSinceProbe;
    uint32_t        _ackCounterTx;   // next counter value this node will send in an ACK
    uint32_t        _ackCounterRx;   // last counter value accepted from the peer
    float           _lastCost[3];

    float _computeCost(AESMode m, SecurityPolicy policy, size_t payloadLenHint = 64) const;
    float _retryFactor(float per) const;
    float _gammaEffective() const;

    void  _ackMac(const ACKMetrics& ackNoMac, uint8_t out[8]) const;

    bool _encryptCBC(const uint8_t* plain, size_t len, const uint8_t* iv,
                     uint8_t* cipher);
    bool _encryptCTR(const uint8_t* plain, size_t len, const uint8_t* nonce,
                     uint8_t* cipher);
    bool _encryptGCM(const uint8_t* plain, size_t len, const uint8_t* nonce,
                     const uint8_t* aad,   size_t aadLen,
                     uint8_t* cipher,      uint8_t tag[16]);

    bool _decryptCBC(const uint8_t* cipher, size_t len, const uint8_t* iv,
                     uint8_t* plain);
    bool _decryptCTR(const uint8_t* cipher, size_t len, const uint8_t* nonce,
                     uint8_t* plain);
    bool _decryptGCM(const uint8_t* cipher, size_t len, const uint8_t* nonce,
                     const uint8_t* aad,    size_t aadLen,
                     const uint8_t  tag[16], uint8_t* plain);

    void _buildNonce(uint32_t seqNum, uint8_t nonce[12]) const;
    uint32_t _deviceId;   // Set from ESP32 MAC in begin()
};
