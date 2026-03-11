#include "LoRa32T.h"
#include <esp_system.h>   // esp_random(), esp_efuse_mac_get_default()
#include <string.h>

// ─── Constexpr storage (required pre-C++17) ──────────────────────────────────
constexpr float LoRa32T::_toaNorm[3];

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════
LoRa32T::LoRa32T()
    : _mode(AESMode::GCM), _dwellCount(0), _switchCount(0), _deviceId(0)
{
    memset(_key,      0, sizeof(_key));
    memset(_lastCost, 0, sizeof(_lastCost));
    _ls = {0.0f, 0.0f, 0.0f, 1.0f, SecurityPolicy::BestEffort};
}

void LoRa32T::begin(const uint8_t key[16],
                             CostWeights weights,
                             StabilityConfig stab)
{
    memcpy(_key, key, 16);
    _w    = weights;
    _stab = stab;

    // Derive a stable 32-bit device ID from the ESP32 base MAC
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    _deviceId = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16)
              | ((uint32_t)mac[4] <<  8) |  (uint32_t)mac[5];

    // Start with a clean SNR estimate of 0 dB (pessimistic; will update fast)
    _ls.snr      = 0.0f;
    _ls.perRadio = 0.0f;
    _ls.perAuth  = 0.0f;
    _ls.battLevel = 1.0f;
}

// ═══════════════════════════════════════════════════════════════════════════
// Transmitter – link metric ingestion
// ═══════════════════════════════════════════════════════════════════════════
void LoRa32T::updateLinkMetrics(const ACKMetrics& ack)
{
    float snrNew    = (float)ack.snrRaw;
    float perNew    = ack.perQ8    / 256.0f;
    float authNew   = ack.authFailQ8 / 256.0f;

    // EWMA α=0.3 for SNR (paper eq. 6); simpler one-shot update for rates
    _ls.snr      = 0.3f * snrNew  + 0.7f * _ls.snr;
    _ls.perRadio = 0.3f * perNew  + 0.7f * _ls.perRadio;
    _ls.perAuth  = 0.3f * authNew + 0.7f * _ls.perAuth;
}

void LoRa32T::updateLinkState(float snrDb, float perRadio,
                                       float perAuth, float battLevel)
{
    _ls.snr       = 0.3f * snrDb    + 0.7f * _ls.snr;
    _ls.perRadio  = 0.3f * perRadio + 0.7f * _ls.perRadio;
    _ls.perAuth   = 0.3f * perAuth  + 0.7f * _ls.perAuth;
    _ls.battLevel = battLevel;  // Direct read – no smoothing needed
}

// ═══════════════════════════════════════════════════════════════════════════
// Cost function
// ═══════════════════════════════════════════════════════════════════════════
float LoRa32T::_retryFactor(float per) const
{
    float p = (per > 0.95f) ? 0.95f : per;
    return 1.0f / (1.0f - p);
}

float LoRa32T::_gammaEffective() const
{
    return _w.gamma0 + _w.kappa * (1.0f - _ls.battLevel);
}

float LoRa32T::_computeCost(AESMode m, SecurityPolicy policy) const
{
    // ── Policy hard gates (paper eq. 21) ────────────────────────────────
    if (policy == SecurityPolicy::Mandatory && m != AESMode::GCM)
        return 1e9f;
    if (policy == SecurityPolicy::Conditional && m == AESMode::CTR && _ls.snr > 0.0f)
        return 1e9f;

    uint8_t idx = static_cast<uint8_t>(m);

    // ── Effective PER (paper eq. 11) ────────────────────────────────────
    float perEff = _ls.perRadio;
    if (m == AESMode::GCM)
        perEff += _ls.perAuth;   // Auth failures are effectively lost packets

    // ── Reliability cost (Flink) ─────────────────────────────────────────
    float fLink = perEff;

    // ── Latency cost (paper eq. 12) ──────────────────────────────────────
    float fLatency = _toaNorm[idx] * _retryFactor(perEff);

    // ── Energy cost (paper eq. 15) ───────────────────────────────────────
    float fEnergy  = _toaNorm[idx];

    float gamma = _gammaEffective();
    return _w.alpha * fLink + _w.beta * fLatency + gamma * fEnergy;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mode selection (Algorithm 1 in the paper)
// ═══════════════════════════════════════════════════════════════════════════
AESMode LoRa32T::selectMode(SecurityPolicy policy)
{
    _ls.policy = policy;

    // Compute cost for all three modes
    for (uint8_t i = 0; i < 3; i++)
        _lastCost[i] = _computeCost(static_cast<AESMode>(i), policy);

    // Find minimum-cost candidate
    AESMode best = AESMode::CTR;
    float bestC  = _lastCost[0];
    for (uint8_t i = 1; i < 3; i++) {
        if (_lastCost[i] < bestC) {
            bestC = _lastCost[i];
            best  = static_cast<AESMode>(i);
        }
    }

    // Already optimal – just increment dwell
    if (best == _mode) {
        _dwellCount++;
        return _mode;
    }

    // Dwell guard: must stay in current mode for minDwellPackets
    if (_dwellCount < _stab.minDwellPackets) {
        _dwellCount++;
        return _mode;
    }

    // Hysteresis: cost improvement must exceed threshold
    float currC = _lastCost[static_cast<uint8_t>(_mode)];
    if ((currC - bestC) < _stab.hystThresh) {
        _dwellCount++;
        return _mode;
    }

    // Commit switch
    _mode       = best;
    _dwellCount = 0;
    _switchCount++;
    return _mode;
}

// ═══════════════════════════════════════════════════════════════════════════
// Nonce construction
// ═══════════════════════════════════════════════════════════════════════════
void LoRa32T::_buildNonce(uint32_t seqNum, uint8_t nonce[12]) const
{
    // [4-byte seqNum] [4-byte deviceId] [4-byte zero padding]
    nonce[0] = (seqNum   >> 24) & 0xFF;
    nonce[1] = (seqNum   >> 16) & 0xFF;
    nonce[2] = (seqNum   >>  8) & 0xFF;
    nonce[3] =  seqNum          & 0xFF;
    nonce[4] = (_deviceId >> 24) & 0xFF;
    nonce[5] = (_deviceId >> 16) & 0xFF;
    nonce[6] = (_deviceId >>  8) & 0xFF;
    nonce[7] =  _deviceId        & 0xFF;
    memset(nonce + 8, 0, 4);
}

// ═══════════════════════════════════════════════════════════════════════════
// Encrypt
// ═══════════════════════════════════════════════════════════════════════════
bool LoRa32T::encrypt(const uint8_t* plain, size_t plainLen,
                               uint32_t seqNum, CryptoFrame& frame)
{
    frame.mode = static_cast<uint8_t>(_mode);
    frame.len  = plainLen;
    memset(frame.tag, 0, 16);

    switch (_mode) {
        case AESMode::CBC: {
            // Random 16-byte IV per packet (NIST recommendation)
            for (int i = 0; i < 16; i++)
                frame.iv[i] = (uint8_t)(esp_random() & 0xFF);
            return _encryptCBC(plain, plainLen, frame.iv, frame.cipher);
        }
        case AESMode::CTR: {
            // 12-byte deterministic nonce, padded to 16 bytes for mbedtls
            _buildNonce(seqNum, frame.iv);          // first 12 bytes used
            memset(frame.iv + 12, 0, 4);
            return _encryptCTR(plain, plainLen, frame.iv, frame.cipher);
        }
        case AESMode::GCM: {
            uint8_t nonce[12];
            _buildNonce(seqNum, nonce);
            memcpy(frame.iv, nonce, 12);
            memset(frame.iv + 12, 0, 4);
            // AAD = seqNum (4 bytes) as a minimal authenticated header
            uint8_t aad[4] = {
                (uint8_t)(seqNum >> 24), (uint8_t)(seqNum >> 16),
                (uint8_t)(seqNum >>  8), (uint8_t) seqNum
            };
            return _encryptGCM(plain, plainLen, nonce, aad, 4,
                                frame.cipher, frame.tag);
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// Decrypt
// ═══════════════════════════════════════════════════════════════════════════
bool LoRa32T::decrypt(const CryptoFrame& frame, uint8_t* plain)
{
    AESMode m = static_cast<AESMode>(frame.mode);
    switch (m) {
        case AESMode::CBC:
            return _decryptCBC(frame.cipher, frame.len, frame.iv, plain);
        case AESMode::CTR:
            return _decryptCTR(frame.cipher, frame.len, frame.iv, plain);
        case AESMode::GCM: {
            // Reconstruct the same AAD the TX used
            // The seqNum is recoverable from the nonce bytes [0..3]
            const uint8_t* nonce = frame.iv;  // first 12 bytes
            uint8_t aad[4] = { nonce[0], nonce[1], nonce[2], nonce[3] };
            return _decryptGCM(frame.cipher, frame.len, nonce, aad, 4,
                                frame.tag, plain);
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// Receiver – ACK builder
// ═══════════════════════════════════════════════════════════════════════════
ACKMetrics LoRa32T::buildACK(int8_t rawSnr,
                                      uint32_t rxTotal, uint32_t rxLost,
                                      uint32_t authFails)
{
    ACKMetrics ack;
    ack.snrRaw = rawSnr;

    float per  = (rxTotal > 0) ? (float)rxLost   / (float)rxTotal : 0.0f;
    float afr  = (rxTotal > 0) ? (float)authFails / (float)rxTotal : 0.0f;

    ack.perQ8      = (uint8_t)(per * 255.0f + 0.5f);
    ack.authFailQ8 = (uint8_t)(afr * 255.0f + 0.5f);
    return ack;
}

// ═══════════════════════════════════════════════════════════════════════════
// AES-CBC  (mbedtls, PKCS#7 padding applied externally by caller if needed)
// ═══════════════════════════════════════════════════════════════════════════
bool LoRa32T::_encryptCBC(const uint8_t* plain, size_t len,
                                   const uint8_t* iv, uint8_t* cipher)
{
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    int ret = mbedtls_aes_setkey_enc(&ctx, _key, 128);
    if (ret) { mbedtls_aes_free(&ctx); return false; }

    uint8_t iv_copy[16];
    memcpy(iv_copy, iv, 16);
    // Encrypt in 16-byte blocks; caller must ensure len is a multiple of 16
    ret = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, len,
                                  iv_copy, plain, cipher);
    mbedtls_aes_free(&ctx);
    return (ret == 0);
}

bool LoRa32T::_decryptCBC(const uint8_t* cipher, size_t len,
                                   const uint8_t* iv, uint8_t* plain)
{
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    int ret = mbedtls_aes_setkey_dec(&ctx, _key, 128);
    if (ret) { mbedtls_aes_free(&ctx); return false; }

    uint8_t iv_copy[16];
    memcpy(iv_copy, iv, 16);
    ret = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, len,
                                  iv_copy, cipher, plain);
    mbedtls_aes_free(&ctx);
    return (ret == 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// AES-CTR  (mbedtls stream cipher; no padding required)
// ═══════════════════════════════════════════════════════════════════════════
bool LoRa32T::_encryptCTR(const uint8_t* plain, size_t len,
                                   const uint8_t* nonce, uint8_t* cipher)
{
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    int ret = mbedtls_aes_setkey_enc(&ctx, _key, 128);
    if (ret) { mbedtls_aes_free(&ctx); return false; }

    uint8_t nc_off_buf = 0;
    uint8_t stream_block[16];
    uint8_t ctr[16];
    memcpy(ctr, nonce, 16);  // nonce already padded to 16 bytes

    ret = mbedtls_aes_crypt_ctr(&ctx, len, (size_t*)&nc_off_buf,
                                  ctr, stream_block, plain, cipher);
    mbedtls_aes_free(&ctx);
    return (ret == 0);
}

bool LoRa32T::_decryptCTR(const uint8_t* cipher, size_t len,
                                   const uint8_t* nonce, uint8_t* plain)
{
    // CTR encrypt == CTR decrypt
    return _encryptCTR(cipher, len, nonce, plain);
}

// ═══════════════════════════════════════════════════════════════════════════
// AES-GCM  (mbedtls AEAD)
// ═══════════════════════════════════════════════════════════════════════════
bool LoRa32T::_encryptGCM(const uint8_t* plain, size_t len,
                                   const uint8_t* nonce,
                                   const uint8_t* aad,  size_t aadLen,
                                   uint8_t* cipher,     uint8_t tag[16])
{
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    int ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, _key, 128);
    if (ret) { mbedtls_gcm_free(&ctx); return false; }

    ret = mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT,
                                     len, nonce, 12,
                                     aad, aadLen,
                                     plain, cipher,
                                     16, tag);
    mbedtls_gcm_free(&ctx);
    return (ret == 0);
}

bool LoRa32T::_decryptGCM(const uint8_t* cipher, size_t len,
                                   const uint8_t* nonce,
                                   const uint8_t* aad,  size_t aadLen,
                                   const uint8_t  tag[16], uint8_t* plain)
{
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    int ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, _key, 128);
    if (ret) { mbedtls_gcm_free(&ctx); return false; }

    // mbedtls_gcm_auth_decrypt returns MBEDTLS_ERR_GCM_AUTH_FAILED on mismatch
    ret = mbedtls_gcm_auth_decrypt(&ctx, len, nonce, 12,
                                    aad, aadLen,
                                    tag, 16,
                                    cipher, plain);
    mbedtls_gcm_free(&ctx);
    return (ret == 0);
}
