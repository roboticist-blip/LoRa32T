#include <Arduino.h>
#include <unity.h>
#include <string.h>

#include "LoRa32T.h"

namespace {

const uint8_t TEST_KEY[16] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
};

void beginController(LoRa32T& controller, float emaAlpha = 1.0f)
{
    controller.begin(TEST_KEY,
                     {0.55f, 0.30f, 0.10f, 0.25f},
                     {0.05f, 0, emaAlpha},
                     {7, 125000.0f, 5, true, false, 8});
}

void assertRoundTrip(LoRa32T& sender, LoRa32T& receiver, AESMode mode,
                     const uint8_t* plain, size_t len, uint32_t sequence)
{
    if (mode == AESMode::GCM) {
        sender.selectMode(SecurityPolicy::Mandatory);
    } else if (mode == AESMode::CTR) {
        sender.selectMode(SecurityPolicy::BestEffort);
    } else {
        ACKMetrics poorAuth = {0, 0, 255, 0, {0}};
        sender.updateLinkMetrics(poorAuth);
        sender.selectMode(SecurityPolicy::Conditional);
    }

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(mode),
                            static_cast<uint8_t>(sender.currentMode()));

    uint8_t cipher[64] = {};
    uint8_t decrypted[64] = {};
    CryptoFrame frame = {};
    frame.cipher = cipher;

    TEST_ASSERT_TRUE(sender.encrypt(plain, len, sequence, frame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(mode), frame.mode);
    TEST_ASSERT_TRUE(receiver.decrypt(frame, decrypted));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(plain, decrypted, len);
}

}  // namespace

void test_frame_headers_and_airtime_follow_mode_overhead()
{
    LoRa32T controller;
    beginController(controller);

    TEST_ASSERT_EQUAL_UINT32(17, controller.frameHeaderSize(AESMode::CTR));
    TEST_ASSERT_EQUAL_UINT32(21, controller.frameHeaderSize(AESMode::CBC));
    TEST_ASSERT_EQUAL_UINT32(33, controller.frameHeaderSize(AESMode::GCM));
    TEST_ASSERT_LESS_THAN(controller.toaMs(AESMode::CBC, 64),
                          controller.toaMs(AESMode::CTR, 64));
    TEST_ASSERT_LESS_THAN(controller.toaMs(AESMode::GCM, 64),
                          controller.toaMs(AESMode::CBC, 64));
}

void test_policy_and_link_metrics_are_applied_correctly()
{
    LoRa32T controller;
    beginController(controller, 0.5f);

    TEST_ASSERT_FALSE(controller.isModeAllowed(AESMode::CTR,
                                               SecurityPolicy::Conditional));
    TEST_ASSERT_FALSE(controller.isModeAllowed(AESMode::CBC,
                                               SecurityPolicy::Mandatory));
    TEST_ASSERT_TRUE(controller.isModeAllowed(AESMode::GCM,
                                              SecurityPolicy::Mandatory));

    ACKMetrics ack = {-8, 128, 64, 0, {0}};
    controller.updateLinkMetrics(ack);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -4.0f, controller.linkState().snr);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 128.0f / 510.0f,
                             controller.linkState().perRadio);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 64.0f / 510.0f,
                             controller.linkState().perAuth);
    TEST_ASSERT_TRUE(controller.isModeAllowed(AESMode::CTR,
                                              SecurityPolicy::Conditional));

    controller.setBatteryLevel(1.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, controller.linkState().battLevel);
    controller.setBatteryLevel(-0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, controller.linkState().battLevel);
}

void test_ack_authentication_rejects_tampering_and_replay()
{
    LoRa32T sender;
    LoRa32T receiver;
    beginController(sender);
    beginController(receiver);

    ACKMetrics ack = sender.buildACK(-12, 10, 2, 1);
    TEST_ASSERT_EQUAL_UINT32(1, ack.counter);
    TEST_ASSERT_TRUE(receiver.verifyACK(ack));
    TEST_ASSERT_FALSE(receiver.verifyACK(ack));

    ACKMetrics tampered = sender.buildACK(-12, 10, 2, 1);
    tampered.perQ8++;
    TEST_ASSERT_FALSE(receiver.verifyACK(tampered));
}

void test_all_aes_modes_round_trip_and_gcm_detects_tag_change()
{
    const uint8_t plain[32] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
    };

    LoRa32T sender;
    LoRa32T receiver;
    beginController(sender);
    beginController(receiver);
    assertRoundTrip(sender, receiver, AESMode::CTR, plain, sizeof(plain), 1);

    beginController(sender);
    beginController(receiver);
    assertRoundTrip(sender, receiver, AESMode::CBC, plain, sizeof(plain), 2);

    beginController(sender);
    beginController(receiver);
    assertRoundTrip(sender, receiver, AESMode::GCM, plain, sizeof(plain), 3);

    uint8_t cipher[32] = {};
    uint8_t decrypted[32] = {};
    CryptoFrame frame = {};
    frame.cipher = cipher;
    TEST_ASSERT_TRUE(sender.encrypt(plain, sizeof(plain), 4, frame));
    frame.tag[0] ^= 0x01;
    TEST_ASSERT_FALSE(receiver.decrypt(frame, decrypted));
}

void setup()
{
    delay(1000);
    UNITY_BEGIN();
    RUN_TEST(test_frame_headers_and_airtime_follow_mode_overhead);
    RUN_TEST(test_policy_and_link_metrics_are_applied_correctly);
    RUN_TEST(test_ack_authentication_rejects_tampering_and_replay);
    RUN_TEST(test_all_aes_modes_round_trip_and_gcm_detects_tag_change);
    UNITY_END();
}

void loop() {}
