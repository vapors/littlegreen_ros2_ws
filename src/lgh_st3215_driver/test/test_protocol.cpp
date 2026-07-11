#include "lgh_st3215_driver/protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace lgh_st3215_driver {


TEST(Protocol, BuildsGenericPingAndReadRequests) {
  const std::vector<std::uint8_t> ping_expected{
      0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  const std::vector<std::uint8_t> read_expected{
      0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
  EXPECT_EQ(buildPingRequest(1), ping_expected);
  EXPECT_EQ(buildReadRequest(1, 0x38, 2), read_expected);
  EXPECT_TRUE(validateFrameChecksum(ping_expected));
  EXPECT_TRUE(validateFrameChecksum(read_expected));
}

TEST(Protocol, ParsesGenericReadReply) {
  std::vector<std::uint8_t> body{0x01, 0x04, 0x00, 0x18, 0x05};
  std::vector<std::uint8_t> frame{0xFF, 0xFF};
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(computeChecksum(body));

  std::uint8_t status = 0xFF;
  std::vector<std::uint8_t> data;
  std::string error;
  ASSERT_TRUE(parseReadReply(frame, 1, 2, status, data, error));
  EXPECT_EQ(status, 0);
  EXPECT_EQ(data, (std::vector<std::uint8_t>{0x18, 0x05}));
  EXPECT_TRUE(error.empty());
}

TEST(Protocol, BuildsReadPresentFeedbackRequest) {
  const auto packet = buildReadPresentFeedbackRequest(1);
  const std::vector<std::uint8_t> expected{
      0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x0F, 0xB1};
  EXPECT_EQ(packet, expected);
}

TEST(Protocol, BuildsBroadcastTorqueEnablePackets) {
  const std::vector<std::uint8_t> disabled_expected{
      0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x28, 0x00, 0xD2};
  const std::vector<std::uint8_t> enabled_expected{
      0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x28, 0x01, 0xD1};
  EXPECT_EQ(buildBroadcastTorqueEnable(false), disabled_expected);
  EXPECT_EQ(buildBroadcastTorqueEnable(true), enabled_expected);
  EXPECT_TRUE(validateFrameChecksum(disabled_expected));
  EXPECT_TRUE(validateFrameChecksum(enabled_expected));
}

TEST(Protocol, BuildsTwelveServoSyncWritePacketWithExpectedSize) {
  std::vector<SyncWriteTarget> targets;
  for (std::uint8_t id = 1; id <= 12; ++id) {
    targets.push_back(SyncWriteTarget{id, 2048, 2000, 100});
  }

  const auto packet = buildSyncWritePositionEx(targets);
  ASSERT_EQ(packet.size(), 104U);
  EXPECT_EQ(packet[0], 0xFF);
  EXPECT_EQ(packet[1], 0xFF);
  EXPECT_EQ(packet[2], 0xFE);
  EXPECT_EQ(packet[3], 0x64);
  EXPECT_EQ(packet[4], 0x83);
  EXPECT_EQ(packet[5], 0x29);
  EXPECT_EQ(packet[6], 0x07);
  EXPECT_TRUE(validateFrameChecksum(packet));
}

TEST(Protocol, PreservesZeroSpeedAndAccelerationInSyncWrite) {
  const std::vector<SyncWriteTarget> targets{
      SyncWriteTarget{5, 1900, 0, 0}};
  const auto packet = buildSyncWritePositionEx(targets);

  // FF FF FE LEN 83 29 07 | ID ACC POS_L POS_H TIME_L TIME_H SPEED_L SPEED_H | CHECKSUM
  ASSERT_EQ(packet.size(), 16U);
  EXPECT_EQ(packet[7], 5);
  EXPECT_EQ(packet[8], 0);   // acceleration
  EXPECT_EQ(packet[13], 0);  // speed low
  EXPECT_EQ(packet[14], 0);  // speed high
  EXPECT_TRUE(validateFrameChecksum(packet));
}

TEST(Protocol, ParsesFullPresentFeedbackReply) {
  // Data 0x38..0x46:
  // position 1990, speed -25, load -321, 12.1 V, 42 C,
  // async flag 0, servo status 0, moving 1, reserved 0,0,
  // current 123 * 6.5 mA.
  std::vector<std::uint8_t> body{
      0x01, 0x11, 0x00,
      0xC6, 0x07,
      0x19, 0x80,
      0x41, 0x05,
      121,
      42,
      0,
      0,
      1,
      0,
      0,
      123, 0};
  std::vector<std::uint8_t> frame{0xFF, 0xFF};
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(computeChecksum(body));

  PresentFeedbackReply reply;
  std::string error;
  ASSERT_TRUE(parsePresentFeedbackReply(frame, 1, reply, error));
  EXPECT_EQ(reply.position_steps, 1990);
  EXPECT_EQ(reply.speed_raw, -25);
  EXPECT_EQ(reply.load_raw, -321);
  EXPECT_EQ(reply.voltage_raw, 121);
  EXPECT_EQ(reply.temperature_c, 42);
  EXPECT_EQ(reply.servo_status, 0);
  EXPECT_TRUE(reply.moving);
  EXPECT_EQ(reply.current_raw, 123);
}

TEST(Protocol, SignedBit15RoundTrip) {
  EXPECT_EQ(decodeSignedBit15(encodeSignedBit15(1234)), 1234);
  EXPECT_EQ(decodeSignedBit15(encodeSignedBit15(-1234)), -1234);
}

TEST(Protocol, DecodesSignedBit10Load) {
  EXPECT_EQ(decodeSignedBit10(321), 321);
  EXPECT_EQ(decodeSignedBit10(static_cast<std::uint16_t>(0x0400U | 321U)), -321);
}

}  // namespace lgh_st3215_driver
