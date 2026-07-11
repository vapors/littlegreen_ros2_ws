#include "lgh_st3215_driver/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lgh_st3215_driver {

std::uint8_t computeChecksum(const std::vector<std::uint8_t>& body) {
  std::uint32_t sum = 0;
  for (const auto byte : body) {
    sum += byte;
  }
  return static_cast<std::uint8_t>(~sum);
}

std::vector<std::uint8_t> buildPacket(
    const std::uint8_t servo_id,
    const std::uint8_t instruction,
    const std::vector<std::uint8_t>& params) {
  if (params.size() > 253U) {
    throw std::invalid_argument("ST3215 packet parameter list is too large");
  }

  const auto length = static_cast<std::uint8_t>(params.size() + 2U);
  std::vector<std::uint8_t> body;
  body.reserve(params.size() + 3U);
  body.push_back(servo_id);
  body.push_back(length);
  body.push_back(instruction);
  body.insert(body.end(), params.begin(), params.end());

  std::vector<std::uint8_t> packet;
  packet.reserve(body.size() + 3U);
  packet.push_back(kHeader);
  packet.push_back(kHeader);
  packet.insert(packet.end(), body.begin(), body.end());
  packet.push_back(computeChecksum(body));
  return packet;
}

std::vector<std::uint8_t> buildPingRequest(const std::uint8_t servo_id) {
  return buildPacket(servo_id, kInstructionPing, {});
}

std::vector<std::uint8_t> buildReadRequest(
    const std::uint8_t servo_id,
    const std::uint8_t address,
    const std::uint8_t length) {
  if (length == 0U) {
    throw std::invalid_argument("ST3215 read length must be nonzero");
  }
  return buildPacket(servo_id, kInstructionRead, {address, length});
}

std::vector<std::uint8_t> buildReadPresentFeedbackRequest(const std::uint8_t servo_id) {
  return buildReadRequest(servo_id, kPresentPositionAddress, kPresentFeedbackReadLength);
}

std::vector<std::uint8_t> buildBroadcastTorqueEnable(const bool enabled) {
  return buildPacket(
      kBroadcastId,
      kInstructionWrite,
      {kTorqueEnableAddress, static_cast<std::uint8_t>(enabled ? 1U : 0U)});
}

std::uint16_t encodeSignedBit15(const int value) {
  const auto magnitude = static_cast<std::uint16_t>(
      std::min(std::abs(value), static_cast<int>(0x7FFF)));
  if (value < 0) {
    return static_cast<std::uint16_t>(magnitude | 0x8000U);
  }
  return magnitude;
}

int decodeSignedBit15(const std::uint16_t raw) {
  if ((raw & 0x8000U) != 0U) {
    return -static_cast<int>(raw & 0x7FFFU);
  }
  return static_cast<int>(raw);
}

int decodeSignedBit10(const std::uint16_t raw) {
  const int magnitude = static_cast<int>(raw & 0x03FFU);
  return (raw & 0x0400U) != 0U ? -magnitude : magnitude;
}

std::vector<std::uint8_t> buildSyncWritePositionEx(
    const std::vector<SyncWriteTarget>& targets) {
  if (targets.empty()) {
    throw std::invalid_argument("SyncWrite requires at least one target");
  }
  if (targets.size() > 31U) {
    throw std::invalid_argument("Too many targets for an 8-bit ST3215 packet length");
  }

  std::vector<std::uint8_t> params;
  params.reserve(2U + targets.size() * 8U);
  params.push_back(kAccelerationAddress);
  params.push_back(kSyncWritePositionDataLength);

  for (const auto& target : targets) {
    const auto encoded_position = encodeSignedBit15(target.position_steps);
    params.push_back(target.servo_id);
    params.push_back(target.acceleration);
    params.push_back(static_cast<std::uint8_t>(encoded_position & 0xFFU));
    params.push_back(static_cast<std::uint8_t>((encoded_position >> 8U) & 0xFFU));
    params.push_back(0x00U);  // goal time low
    params.push_back(0x00U);  // goal time high
    params.push_back(static_cast<std::uint8_t>(target.speed & 0xFFU));
    params.push_back(static_cast<std::uint8_t>((target.speed >> 8U) & 0xFFU));
  }

  return buildPacket(kBroadcastId, kInstructionSyncWrite, params);
}

bool validateFrameChecksum(const std::vector<std::uint8_t>& frame) {
  if (frame.size() < 6U || frame[0] != kHeader || frame[1] != kHeader) {
    return false;
  }

  std::uint32_t sum = 0;
  for (std::size_t i = 2; i < frame.size(); ++i) {
    sum += frame[i];
  }
  return static_cast<std::uint8_t>(sum & 0xFFU) == 0xFFU;
}


bool parseReadReply(
    const std::vector<std::uint8_t>& frame,
    const std::uint8_t expected_id,
    const std::size_t expected_data_length,
    std::uint8_t& status_error,
    std::vector<std::uint8_t>& data,
    std::string& error) {
  const std::size_t expected_frame_size = expected_data_length + 6U;
  if (frame.size() != expected_frame_size) {
    error = "read reply has unexpected frame size";
    return false;
  }
  if (frame[0] != kHeader || frame[1] != kHeader) {
    error = "invalid frame header";
    return false;
  }
  if (frame[2] != expected_id) {
    error = "reply servo ID does not match request";
    return false;
  }
  if (frame[3] != static_cast<std::uint8_t>(expected_data_length + 2U)) {
    error = "unexpected reply length field";
    return false;
  }
  if (!validateFrameChecksum(frame)) {
    error = "checksum mismatch";
    return false;
  }
  status_error = frame[4];
  data.assign(frame.begin() + 5, frame.end() - 1);
  error.clear();
  return true;
}

bool parsePresentFeedbackReply(
    const std::vector<std::uint8_t>& frame,
    const std::uint8_t expected_id,
    PresentFeedbackReply& reply,
    std::string& error) {
  constexpr std::size_t kExpectedFrameSize =
      static_cast<std::size_t>(kPresentFeedbackReadLength) + 6U;
  constexpr std::uint8_t kExpectedLengthField = kPresentFeedbackReadLength + 2U;

  if (frame.size() != kExpectedFrameSize) {
    error = "present-feedback reply must be exactly 21 bytes";
    return false;
  }
  if (frame[0] != kHeader || frame[1] != kHeader) {
    error = "invalid frame header";
    return false;
  }
  if (frame[2] != expected_id) {
    error = "reply servo ID does not match request";
    return false;
  }
  if (frame[3] != kExpectedLengthField) {
    error = "unexpected reply length field";
    return false;
  }
  if (!validateFrameChecksum(frame)) {
    error = "checksum mismatch";
    return false;
  }

  const auto word_at = [&frame](const std::size_t data_offset) {
    const std::size_t i = 5U + data_offset;
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(frame[i]) |
        (static_cast<std::uint16_t>(frame[i + 1U]) << 8U));
  };

  reply.servo_id = frame[2];
  reply.status_error = frame[4];
  reply.position_steps = decodeSignedBit15(word_at(0));
  reply.speed_raw = decodeSignedBit15(word_at(2));
  reply.load_raw = decodeSignedBit10(word_at(4));
  reply.voltage_raw = frame[11];       // 0x3E, units 0.1 V
  reply.temperature_c = frame[12];     // 0x3F, units deg C
  reply.servo_status = frame[14];      // 0x41
  reply.moving = frame[15] != 0U;      // 0x42
  reply.current_raw = decodeSignedBit15(word_at(13));  // 0x45..0x46, units 6.5 mA
  error.clear();
  return true;
}

}  // namespace lgh_st3215_driver
