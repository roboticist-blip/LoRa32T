#pragma once

#include <Arduino.h>
#include <mbedtls/aes.h>
#include <mbedtls/gcm.h>

// ─── AES Mode Identifiers ───────────────────────────────────────────────────
enum class AESMode : uint8_t {
    CTR = 0,
    CBC = 1,
    GCM = 2
};

// ─── Security Policy ────────────────────────────────────────────────────────
enum class SecurityPolicy : uint8_t {
    BestEffort  = 0,  // Cost-driven, all modes permitted
    Conditional = 1,  // GCM preferred; CTR only when SNR < 0 dB
    Mandatory   = 2   // GCM always, regardless of channel
};

// ─── Link State (populated from receiver ACK) ───────────────────────────────
struct LinkState {
    float    snr;           // Smoothed SNR (dB), EWMA α=0.3
    float    perRadio;      // Radio-layer packet error rate [0,1]
    float    perAuth;       // GCM GMAC failure rate [0,1], 0 for CTR/CBC
    float    battLevel;     // Normalised battery [0,1]
    SecurityPolicy policy;
};

// ─── Compact ACK payload sent by RX → TX ────────────────────────────────────
struct ACKMetrics {
    int8_t   snrRaw;        // Raw SNR from SX1278, signed dB
    uint8_t  perQ8;         // PER × 256 (fixed-point, range 0-255)
    uint8_t  authFailQ8;    // Auth failure rate × 256
};                          // 3 bytes total

// ─── Encryption context returned from encrypt/decrypt calls ─────────────────
struct CryptoFrame {
    uint8_t  mode;          // AESMode cast to uint8_t (placed in packet header)
    uint8_t  iv[16];        // IV/nonce (16 bytes for CBC, 12 for CTR/GCM)
    uint8_t  tag[16];       // GCM auth tag (zero-filled for CBC/CTR)
    uint8_t* cipher;        // Caller-supplied output buffer
    size_t   len;           // Ciphertext length (== plaintext length for CTR/GCM)
};

// ─── Cost-function weights ───────────────────────────────────────────────────
struct CostWeights {
    float alpha;   // Reliability weight  (default 0.55)
    float beta;    // Latency weight      (default 0.30)
    float gamma0;  // Base energy weight  (default 0.10)
    float kappa;   // Battery sensitivity (default 0.25)
};

// ─── Stability parameters ────────────────────────────────────────────────────
struct StabilityConfig {
    float    hystThresh;       // Min cost improvement to switch (default 0.05)
    uint16_t minDwellPackets;  // Packets before reconsidering mode (default 20)
};

// ═══════════════════════════════════════════════════════════════════════════
// LoRa32T – main class
// ═══════════════════════════════════════════════════════════════════════════
class LoRa32T {
public:
    // ── Construction / Initialisation ────────────────────────────────────
    LoRa32T();

    // Supply 16-byte pre-shared AES-128 key.
    // Call once during setup().
    void begin(const uint8_t key[16],
               CostWeights    weights  = {0.55f, 0.30f, 0.10f, 0.25f},
               StabilityConfig stab    = {0.05f, 20});

    // ── Transmitter API ──────────────────────────────────────────────────

    // Ingest a new ACK from the receiver; updates internal EWMA state.
    void updateLinkMetrics(const ACKMetrics& ack);

    // Directly update link state (alternative when you parse ACK yourself).
    void updateLinkState(float snrDb, float perRadio, float perAuth, float battLevel);

    // Evaluate cost function + stability guards → returns the selected mode.
    AESMode selectMode(SecurityPolicy policy = SecurityPolicy::BestEffort);

    // Encrypt plaintext into frame.cipher (caller allocates buffer ≥ plainLen).
    // Fills frame.iv, frame.tag, frame.mode.
    // seqNum is used as part of CTR/GCM nonce for replay protection.
    bool encrypt(const uint8_t* plain, size_t plainLen,
                 uint32_t seqNum, CryptoFrame& frame);

    // ── Receiver API ─────────────────────────────────────────────────────

    // Decrypt an incoming CryptoFrame into plain[] (caller allocates ≥ frame.len).
    // Returns false on GCM auth failure or bad structure.
    bool decrypt(const CryptoFrame& frame, uint8_t* plain);

    // Build a compact ACK from receiver-side measurements.
    // Call after each received packet; pass the raw SX1278 SNR and running counters.
    ACKMetrics buildACK(int8_t rawSnr,
                        uint32_t rxTotal, uint32_t rxLost,
                        uint32_t authFails);

    // ── Diagnostics ──────────────────────────────────────────────────────

    // Current smoothed link state (read-only).
    const LinkState& linkState() const { return _ls; }

    // Currently active encryption mode.
    AESMode currentMode() const { return _mode; }

    // Number of mode switches since begin().
    uint32_t modeSwitchCount() const { return _switchCount; }

    // Per-mode cost from the last selectMode() call (for logging/debug).
    float lastCost(AESMode m) const { return _lastCost[static_cast<uint8_t>(m)]; }

private:
    // ── Internal state ───────────────────────────────────────────────────
    uint8_t         _key[16];
    CostWeights     _w;
    StabilityConfig _stab;
    LinkState       _ls;
    AESMode         _mode;
    uint16_t        _dwellCount;
    uint32_t        _switchCount;
    float           _lastCost[3];

    // ── Cost function internals ───────────────────────────────────────────
    // Normalised ToA relative to CBC baseline (CBC=1.00, CTR=0.80, GCM=0.95)
    static constexpr float _toaNorm[3] = {0.80f, 1.00f, 0.95f};

    float _computeCost(AESMode m, SecurityPolicy policy) const;
    float _retryFactor(float per) const;
    float _gammaEffective() const;

    // ── Crypto helpers ───────────────────────────────────────────────────
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

    // Build 12-byte nonce: [4-byte seqNum | 4-byte deviceId | 4-byte counter]
    void _buildNonce(uint32_t seqNum, uint8_t nonce[12]) const;

    uint32_t _deviceId;   // Set from ESP32 MAC in begin()
};
