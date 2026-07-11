#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lgh_st3215_driver {

constexpr std::uint8_t kHeader = 0xFF;
constexpr std::uint8_t kBroadcastId = 0xFE;
constexpr std::uint8_t kInstructionPing = 0x01;
constexpr std::uint8_t kInstructionRead = 0x02;
constexpr std::uint8_t kInstructionWrite = 0x03;
constexpr std::uint8_t kInstructionSyncWrite = 0x83;

constexpr std::uint8_t kTorqueEnableAddress = 0x28;
constexpr std::uint8_t kAccelerationAddress = 0x29;
constexpr std::uint8_t kPresentPositionAddress = 0x38;
// STS3215 contiguous feedback window 0x38..0x46 inclusive:
// position, speed, load, voltage, temperature, flags/status/moving, current.
constexpr std::uint8_t kPresentFeedbackReadLength = 15;
constexpr std::uint8_t kSyncWritePositionDataLength = 7;

struct SyncWriteTarget {
  std::uint8_t servo_id{0};
  std::int16_t position_steps{0};
  std::uint16_t speed{0};
  std::uint8_t acceleration{0};
};

struct PresentFeedbackReply {
  std::uint8_t servo_id{0};
  std::uint8_t status_error{0};
  int position_steps{0};
  int speed_raw{0};
  int load_raw{0};
  std::uint8_t voltage_raw{0};
  std::uint8_t temperature_c{0};
  std::uint8_t servo_status{0};
  bool moving{false};
  int current_raw{0};
};

std::uint8_t computeChecksum(const std::vector<std::uint8_t>& body);
std::vector<std::uint8_t> buildPacket(
    std::uint8_t servo_id,
    std::uint8_t instruction,
    const std::vector<std::uint8_t>& params);
std::vector<std::uint8_t> buildPingRequest(std::uint8_t servo_id);
std::vector<std::uint8_t> buildReadRequest(
    std::uint8_t servo_id, std::uint8_t address, std::uint8_t length);
std::vector<std::uint8_t> buildReadPresentFeedbackRequest(std::uint8_t servo_id);
std::vector<std::uint8_t> buildBroadcastTorqueEnable(bool enabled);
std::vector<std::uint8_t> buildSyncWritePositionEx(
    const std::vector<SyncWriteTarget>& targets);

int decodeSignedBit15(std::uint16_t raw);
std::uint16_t encodeSignedBit15(int value);
int decodeSignedBit10(std::uint16_t raw);

bool validateFrameChecksum(const std::vector<std::uint8_t>& frame);
bool parseReadReply(
    const std::vector<std::uint8_t>& frame,
    std::uint8_t expected_id,
    std::size_t expected_data_length,
    std::uint8_t& status_error,
    std::vector<std::uint8_t>& data,
    std::string& error);
bool parsePresentFeedbackReply(
    const std::vector<std::uint8_t>& frame,
    std::uint8_t expected_id,
    PresentFeedbackReply& reply,
    std::string& error);

}  // namespace lgh_st3215_driver
