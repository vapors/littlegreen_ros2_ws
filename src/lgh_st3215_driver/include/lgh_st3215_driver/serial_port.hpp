#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace lgh_st3215_driver {

enum class IoStatus {
  kOk,
  kTimeout,
  kChecksumError,
  kMalformedFrame,
  kWrongServoId,
  kIoError,
};

struct FrameReadResult {
  IoStatus status{IoStatus::kIoError};
  std::vector<std::uint8_t> frame;
  std::string message;
};

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  void openPort(const std::string& path, int baud);
  void closePort();
  bool isOpen() const noexcept;
  int fd() const noexcept;

  IoStatus writeAll(
      const std::vector<std::uint8_t>& data,
      std::chrono::milliseconds timeout,
      std::string& error);

  FrameReadResult readFrame(
      std::uint8_t expected_id,
      std::chrono::milliseconds timeout);

  void flushInput();
  const std::string& lockPath() const noexcept;

 private:
  void acquireLock(const std::string& path);
  void releaseLock();
  bool waitFor(short events, std::chrono::steady_clock::time_point deadline, std::string& error);
  bool extractFrame(std::vector<std::uint8_t>& frame, std::string& error);

  int fd_{-1};
  int lock_fd_{-1};
  std::string path_;
  std::string lock_path_;
  std::vector<std::uint8_t> rx_buffer_;
};

const char* toString(IoStatus status) noexcept;

}  // namespace lgh_st3215_driver
