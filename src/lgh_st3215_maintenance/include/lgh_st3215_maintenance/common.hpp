#pragma once

#include "lgh_st3215_driver/serial_port.hpp"
#include "lgh_st3215_driver/protocol.hpp"
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace lgh_st3215_maintenance {
constexpr int kPass = 0;
constexpr int kTestFail = 2;
constexpr int kRefused = 3;
constexpr int kTimeout = 4;
constexpr int kConfig = 5;
constexpr int kHardware = 6;
constexpr int kInternal = 70;

inline int parseInt(const std::string& value) {
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed, 0);
  if (consumed != value.size()) throw std::invalid_argument("invalid integer: " + value);
  return result;
}

inline std::string hexBytes(const std::vector<std::uint8_t>& data) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < data.size(); ++i) {
    if (i) out << ' ';
    out << std::setw(2) << static_cast<unsigned int>(data[i]);
  }
  return out.str();
}

inline bool transact(
    lgh_st3215_driver::SerialPort& port,
    const std::vector<std::uint8_t>& request,
    std::uint8_t id,
    int write_timeout_ms,
    int read_timeout_ms,
    lgh_st3215_driver::FrameReadResult& reply,
    std::string& error) {
  port.flushInput();
  const auto write_status = port.writeAll(
      request, std::chrono::milliseconds(write_timeout_ms), error);
  if (write_status != lgh_st3215_driver::IoStatus::kOk) return false;
  reply = port.readFrame(id, std::chrono::milliseconds(read_timeout_ms));
  if (reply.status != lgh_st3215_driver::IoStatus::kOk) {
    error = reply.message;
    return false;
  }
  error.clear();
  return true;
}

inline int exceptionExit(const std::exception& error) {
  const std::string text = error.what();
  return text.find("ownership refused") != std::string::npos ? kRefused : kHardware;
}
}  // namespace lgh_st3215_maintenance
