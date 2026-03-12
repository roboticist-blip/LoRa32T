#pragma once

#include <Arduino.h>
#include <mbedtls/aes.h>
#include <mbedtls/gcm.h>

enum class AESMode : uint8_t {
    CTR = 0,
    CBC = 1,
    GCM = 2
};

enum class SecurityPolicy : uint8_t {
    BestEffort  = 0,  // Cost-driven, all modes permitted
    Conditional = 1,  // GCM preferred; CTR only when SNR < 0 dB
    Mandatory   = 2   // GCM always, regardless of channel
};

struct LinkState {
    float    snr;
    float    perRadio;
    float    perAuth;
    float    battLevel;
    SecurityPolicy policy;
};

struct ACKMetrics {
    int8_t   snrRaw;
    uint8_t  perQ8;
    uint8_t  authFailQ8;
};

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
    float    hystThresh;
    uint16_t minDwellPackets;
};


class LoRa32T {
public:
    LoRa32T();
    void begin(const uint8_t key[16],
               CostWeights    weights  = {0.55f, 0.30f, 0.10f, 0.25f},
               StabilityConfig stab    = {0.05f, 20});
    void updateLinkMetrics(const ACKMetrics& ack);
    void updateLinkState(float snrDb, float perRadio, float perAuth, float battLevel);
    AESMode selectMode(SecurityPolicy policy = SecurityPolicy::BestEffort);
    bool encrypt(const uint8_t* plain, size_t plainLen,
                 uint32_t seqNum, CryptoFrame& frame);
    bool decrypt(const CryptoFrame& frame, uint8_t* plain);
    ACKMetrics buildACK(int8_t rawSnr,
                        uint32_t rxTotal, uint32_t rxLost,
                        uint32_t authFails);
    const LinkState& linkState() const { return _ls; }
    AESMode currentMode() const { return _mode; }
    uint32_t modeSwitchCount() const { return _switchCount; }
    float lastCost(AESMode m) const { return _lastCost[static_cast<uint8_t>(m)]; }

private:
    uint8_t         _key[16];
    CostWeights     _w;
    StabilityConfig _stab;
    LinkState       _ls;
    AESMode         _mode;
    uint16_t        _dwellCount;
    uint32_t        _switchCount;
    float           _lastCost[3];

    // Normalised ToA relative to CBC baseline (CBC=1.00, CTR=0.80, GCM=0.95)
    static constexpr float _toaNorm[3] = {0.80f, 1.00f, 0.95f};

    float _computeCost(AESMode m, SecurityPolicy policy) const;
    float _retryFactor(float per) const;
    float _gammaEffective() const;

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
