#include "LoRa32T.h"
#include <esp_system.h>
#include <string.h>
#include <math.h>

LoRa32T::LoRa32T()
    : _mode(AESMode::GCM), _dwellCount(0), _switchCount(0),
      _packetsSinceProbe(0), _ackCounterTx(0), _ackCounterRx(0), _deviceId(0)
{
    memset(_key,      0, sizeof(_key));
    memset(_lastCost, 0, sizeof(_lastCost));
    _ls = {0.0f, 0.0f, 0.0f, 1.0f, SecurityPolicy::BestEffort};
}

void LoRa32T::begin(const uint8_t key[16],
                             CostWeights weights,
                             StabilityConfig stab,
                             RadioConfig radio)
{
    memcpy(_key, key, 16);
    _w     = weights;
    _stab  = stab;
    _radio = radio;
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    _deviceId = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16)
              | ((uint32_t)mac[4] <<  8) |  (uint32_t)mac[5];
    _ls.snr      = 0.0f;
    _ls.perRadio = 0.0f;
    _ls.perAuth  = 0.0f;
    _ls.battLevel = 1.0f;
}

void LoRa32T::updateLinkMetrics(const ACKMetrics& ack)
{
    // Metrics arrive from the receiver already carrying real information;
    // apply the EMA exactly once here. Battery is intentionally NOT part
    // of this path (see setBatteryLevel) — it is a direct local reading,
    // not a filtered link estimate, and re-smoothing it here was the
    // double-smoothing bug in M-7.
    float snrNew  = (float)ack.snrRaw;
    float perNew  = ack.perQ8       / 255.0f;
    float authNew = ack.authFailQ8  / 255.0f;
    float a = _stab.emaAlpha;
    _ls.snr      = a * snrNew  + (1.0f - a) * _ls.snr;
    _ls.perRadio = a * perNew  + (1.0f - a) * _ls.perRadio;
    _ls.perAuth  = a * authNew + (1.0f - a) * _ls.perAuth;
}

void LoRa32T::setBatteryLevel(float battLevel)
{
    // Direct measurement — stored as-is, never fed back through the EMA.
    _ls.battLevel = constrain(battLevel, 0.0f, 1.0f);
}

void LoRa32T::noteCrcResult(bool crcOk)
{
    // Generic corruption signal usable in every mode, not just GCM (M-3).
    // Folded into perAuth with the same EMA so the reliability term stays
    // informative while the controller is parked in CTR/CBC.
    float a = _stab.emaAlpha;
    float sample = crcOk ? 0.0f : 1.0f;
    _ls.perAuth = a * sample + (1.0f - a) * _ls.perAuth;
}

bool LoRa32T::shouldProbeGCM()
{
    _packetsSinceProbe++;
    uint16_t interval = _stab.minDwellPackets > 0 ? _stab.minDwellPackets : 20;
    if (_mode != AESMode::GCM && _packetsSinceProbe >= interval) {
        _packetsSinceProbe = 0;
        return true;
    }
    return false;
}

float LoRa32T::_retryFactor(float per) const
{
    float p = (per > 0.95f) ? 0.95f : per;
    return 1.0f / (1.0f - p);
}

float LoRa32T::_gammaEffective() const
{
    return _w.gamma0 + _w.kappa * (1.0f - _ls.battLevel);
}

size_t LoRa32T::frameHeaderSize(AESMode m) const
{
    // mode(1) + iv/nonce + [tag(16) only for GCM] + seq(4)
    // CBC needs a full random 16-byte IV; CTR/GCM use a 12-byte nonce.
    size_t ivLen  = (m == AESMode::CBC) ? 16 : 12;
    size_t tagLen = (m == AESMode::GCM) ? 16 : 0;
    return 1 + ivLen + tagLen + 4;
}

float LoRa32T::toaMs(AESMode m, size_t payloadLen) const
{
    // Semtech LoRa time-on-air formula (paper eq. 1-3), driven by the
    // ACTUAL per-mode frame size instead of a hardcoded ratio (M-1).
    const float bw = _radio.bwHz;
    const float tSym = (1u << _radio.sf) / bw * 1000.0f; // ms

    size_t pl = payloadLen + frameHeaderSize(m);
    int de = _radio.lowDataRateOptimize ? 1 : 0;
    int h  = _radio.explicitHeader ? 0 : 1;

    float numerator = 8.0f * (float)pl - 4.0f * _radio.sf + 28.0f + 16.0f - 20.0f * h;
    float denom      = 4.0f * (_radio.sf - 2.0f * de);
    float nPayload = 8.0f + fmaxf(ceilf(numerator / denom) * (_radio.cr + 4), 0.0f);

    float tPreamble = (_radio.preambleSymbols + 4.25f) * tSym;
    return tPreamble + nPayload * tSym;
}

float LoRa32T::_computeCost(AESMode m, SecurityPolicy policy, size_t payloadLenHint) const
{
    if (!isModeAllowed(m, policy))
        return 1e9f;

    uint8_t idx = static_cast<uint8_t>(m);

    float perEff = _ls.perRadio;
    if (m == AESMode::GCM)
        perEff += _ls.perAuth;

    // Reliability cost (Flink)
    float fLink = perEff;

    // Airtime normalised to CBC baseline, computed live from real frame
    // sizes rather than a hardcoded {0.80, 1.00, 0.95} table (M-1).
    float toaCBC = toaMs(AESMode::CBC, payloadLenHint);
    float toaM   = toaMs(m,            payloadLenHint);
    float toaNorm = (toaCBC > 0.0f) ? (toaM / toaCBC) : 1.0f;

    // Latency cost (paper eq. 12) 
    float fLatency = toaNorm * _retryFactor(perEff);

    // Energy cost (paper eq. 15) ─
    // NOTE (M-2): fEnergy and the airtime component of fLatency share the
    // same physical basis (toaNorm), so beta and gamma are not
    // independently identifiable from this term alone. Until energy is
    // measured directly (e.g. INA219 current draw per mode, including the
    // AES computation cost), treat alpha and a combined (beta, gamma) as
    // the two effectively free parameters of this model, not three.
    float fEnergy  = toaNorm;

    float gamma = _gammaEffective();
    return _w.alpha * fLink + _w.beta * fLatency + gamma * fEnergy;
}

bool LoRa32T::isModeAllowed(AESMode frameMode, SecurityPolicy policy) const
{
    if (policy == SecurityPolicy::Mandatory && frameMode != AESMode::GCM)
        return false;
    // Conditional: CTR only when SNR is genuinely poor. Boundary fixed to
    // >= 0 dB so it matches the documented behaviour exactly at the
    // quantised 0 dB value that the receive path actually produces (M-6).
    if (policy == SecurityPolicy::Conditional && frameMode == AESMode::CTR && _ls.snr >= 0.0f)
        return false;
    return true;
}

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
    float currC = _lastCost[static_cast<uint8_t>(_mode)];
    if (currC >= 1e8f && best != _mode) {
        _mode = best;
        _dwellCount = 0;
        _switchCount++;
        return _mode;
    }
    if (best == _mode) {
        _dwellCount++;
        return _mode;
    }
    if (_dwellCount < _stab.minDwellPackets) {
        _dwellCount++;
        return _mode;
    }
    // Relative hysteresis (M-5): a fixed cost margin, not a fixed absolute
    // difference, so switching sensitivity stays constant across the cost
    // range instead of drifting between ~7% and ~16% depending on the
    // operating point.
    float bestSafe = (bestC > 1e-6f) ? bestC : 1e-6f;
    if (((currC - bestC) / bestSafe) < _stab.hystThresh) {
        _dwellCount++;
        return _mode;
    }
    _mode       = best;
    _dwellCount = 0;
    _switchCount++;
    return _mode;
}


void LoRa32T::_buildNonce(uint32_t seqNum, uint8_t nonce[12]) const
{
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

bool LoRa32T::encrypt(const uint8_t* plain, size_t plainLen,
                               uint32_t seqNum, CryptoFrame& frame)
{
    frame.mode = static_cast<uint8_t>(_mode);
    frame.len  = plainLen;
    memset(frame.tag, 0, 16);
    memset(frame.iv,  0, 16);

    // Mode byte + seqNum are bound into the GCM AAD (S-2): a receiver
    // that decrypts as GCM is thereby cryptographically guaranteed the
    // sender also selected GCM, so an attacker flipping the cleartext
    // mode byte in transit cannot silently downgrade an authenticated
    // frame — the tag simply won't verify.
    switch (_mode) {
        case AESMode::CBC: {
            for (int i = 0; i < 16; i++)
                frame.iv[i] = (uint8_t)(esp_random() & 0xFF);
            return _encryptCBC(plain, plainLen, frame.iv, frame.cipher);
        }
        case AESMode::CTR: {
            _buildNonce(seqNum, frame.iv);
            return _encryptCTR(plain, plainLen, frame.iv, frame.cipher);
        }
        case AESMode::GCM: {
            uint8_t nonce[12];
            _buildNonce(seqNum, nonce);
            memcpy(frame.iv, nonce, 12);
            uint8_t aad[5] = {
                frame.mode,
                (uint8_t)(seqNum >> 24), (uint8_t)(seqNum >> 16),
                (uint8_t)(seqNum >>  8), (uint8_t) seqNum
            };
            return _encryptGCM(plain, plainLen, nonce, aad, 5,
                                frame.cipher, frame.tag);
        }
    }
    return false;
}

bool LoRa32T::decrypt(const CryptoFrame& frame, uint8_t* plain)
{
    AESMode m = static_cast<AESMode>(frame.mode);
    switch (m) {
        case AESMode::CBC:
            return _decryptCBC(frame.cipher, frame.len, frame.iv, plain);
        case AESMode::CTR:
            return _decryptCTR(frame.cipher, frame.len, frame.iv, plain);
        case AESMode::GCM: {
            const uint8_t* nonce = frame.iv;
            // Reconstruct AAD from the same fields the sender bound in.
            // seqNum isn't carried separately here — callers that need
            // seqNum-in-AAD must pass it via the frame's own seq field
            // when parsing the wire packet (see RX example).
            uint8_t aad[5] = { frame.mode, nonce[0], nonce[1], nonce[2], nonce[3] };
            return _decryptGCM(frame.cipher, frame.len, nonce, aad, 5,
                                frame.tag, plain);
        }
    }
    return false;
}

void LoRa32T::_ackMac(const ACKMetrics& ackNoMac, uint8_t out[8]) const
{
    uint8_t buf[7];
    buf[0] = (uint8_t)ackNoMac.snrRaw;
    buf[1] = ackNoMac.perQ8;
    buf[2] = ackNoMac.authFailQ8;
    buf[3] = (uint8_t)(ackNoMac.counter >> 24);
    buf[4] = (uint8_t)(ackNoMac.counter >> 16);
    buf[5] = (uint8_t)(ackNoMac.counter >>  8);
    buf[6] = (uint8_t)(ackNoMac.counter);

    uint8_t full[32];
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    mbedtls_md_hmac(info, _key, sizeof(_key), buf, sizeof(buf), full);
    memcpy(out, full, 8);   // truncated tag; sufficient given the small ACK
}

ACKMetrics LoRa32T::buildACK(int8_t rawSnr,
                                      uint32_t rxTotal, uint32_t rxLost,
                                      uint32_t authFails)
{
    ACKMetrics ack;
    ack.snrRaw = rawSnr;

    float per  = (rxTotal > 0) ? (float)rxLost    / (float)rxTotal : 0.0f;
    float afr  = (rxTotal > 0) ? (float)authFails / (float)rxTotal : 0.0f;
    per = constrain(per, 0.0f, 1.0f);
    afr = constrain(afr, 0.0f, 1.0f);

    ack.perQ8      = (uint8_t)(per * 255.0f + 0.5f);
    ack.authFailQ8 = (uint8_t)(afr * 255.0f + 0.5f);
    ack.counter    = ++_ackCounterTx;
    _ackMac(ack, ack.mac);
    return ack;
}

bool LoRa32T::verifyACK(const ACKMetrics& ack)
{
    // Reject stale or replayed counters (S-4). A fresh device pairing
    // should reset _ackCounterRx (e.g. on a mode-0/handshake packet);
    // this implementation takes the simple monotonic-counter approach
    // the audit's fix recommends, not a full session-resync protocol.
    if (ack.counter <= _ackCounterRx)
        return false;

    uint8_t expected[8];
    _ackMac(ack, expected);
    uint8_t diff = 0;
    for (int i = 0; i < 8; i++) diff |= (expected[i] ^ ack.mac[i]);
    if (diff != 0)
        return false;

    _ackCounterRx = ack.counter;
    return true;
}

bool LoRa32T::_encryptCBC(const uint8_t* plain, size_t len,
                                   const uint8_t* iv, uint8_t* cipher)
{
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    int ret = mbedtls_aes_setkey_enc(&ctx, _key, 128);
    if (ret) { mbedtls_aes_free(&ctx); return false; }

    uint8_t iv_copy[16];
    memcpy(iv_copy, iv, 16);
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

bool LoRa32T::_encryptCTR(const uint8_t* plain, size_t len,
                                   const uint8_t* nonce, uint8_t* cipher)
{
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    int ret = mbedtls_aes_setkey_enc(&ctx, _key, 128);
    if (ret) { mbedtls_aes_free(&ctx); return false; }

    size_t nc_off = 0;
    uint8_t stream_block[16];
    uint8_t ctr[16];
    memcpy(ctr, nonce, 16);

    ret = mbedtls_aes_crypt_ctr(&ctx, len, &nc_off,
                                  ctr, stream_block, plain, cipher);
    mbedtls_aes_free(&ctx);
    return (ret == 0);
}

bool LoRa32T::_decryptCTR(const uint8_t* cipher, size_t len,
                                   const uint8_t* nonce, uint8_t* plain)
{
    return _encryptCTR(cipher, len, nonce, plain);
}

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

    ret = mbedtls_gcm_auth_decrypt(&ctx, len, nonce, 12,
                                    aad, aadLen,
                                    tag, 16,
                                    cipher, plain);
    mbedtls_gcm_free(&ctx);
    return (ret == 0);
}
