#include "lgh_st3215_driver/serial_port.hpp"

#include "lgh_st3215_driver/protocol.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <cctype>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace lgh_st3215_driver {
namespace {

speed_t baudToTermios(const int baud) {
  switch (baud) {
    case 115200:
      return B115200;
#ifdef B500000
    case 500000:
      return B500000;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
    default:
      throw std::invalid_argument("Unsupported UART baud rate: " + std::to_string(baud));
  }
}

std::string lockPathForDevice(const std::string& path) {
  std::string token;
  token.reserve(path.size());
  for (const char ch : path) {
    token.push_back(std::isalnum(static_cast<unsigned char>(ch)) ? ch : '_');
  }
  return "/tmp/lgh_st3215" + token + ".lock";
}

std::chrono::milliseconds remainingMs(const std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return std::chrono::milliseconds(0);
  }
  auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
  if (remaining.count() == 0) {
    remaining = std::chrono::milliseconds(1);
  }
  return remaining;
}

}  // namespace

SerialPort::~SerialPort() { closePort(); }

void SerialPort::openPort(const std::string& path, const int baud) {
  closePort();
  acquireLock(path);

  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    const std::string error = std::strerror(errno);
    releaseLock();
    throw std::runtime_error("Failed to open " + path + ": " + error);
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    const std::string error = std::strerror(errno);
    closePort();
    throw std::runtime_error("tcgetattr failed for " + path + ": " + error);
  }

  ::cfmakeraw(&tty);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const speed_t speed = baudToTermios(baud);
  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    const std::string error = std::strerror(errno);
    closePort();
    throw std::runtime_error("Failed to set UART speed on " + path + ": " + error);
  }

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const std::string error = std::strerror(errno);
    closePort();
    throw std::runtime_error("tcsetattr failed for " + path + ": " + error);
  }

  path_ = path;
  rx_buffer_.clear();
  ::tcflush(fd_, TCIOFLUSH);
}

void SerialPort::closePort() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  path_.clear();
  rx_buffer_.clear();
  releaseLock();
}

void SerialPort::acquireLock(const std::string& path) {
  lock_path_ = lockPathForDevice(path);
  lock_fd_ = ::open(lock_path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
  if (lock_fd_ < 0) {
    const std::string error = std::strerror(errno);
    lock_path_.clear();
    throw std::runtime_error("Failed to open ST3215 ownership lock: " + error);
  }
  if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    const std::string error = std::strerror(errno);
    ::close(lock_fd_);
    lock_fd_ = -1;
    const std::string held_path = lock_path_;
    lock_path_.clear();
    throw std::runtime_error(
        "ST3215 UART ownership refused: " + path + " is already locked via " +
        held_path + " (" + error + ")");
  }
  const std::string owner = std::to_string(static_cast<long long>(::getpid())) + "\n";
  (void)::ftruncate(lock_fd_, 0);
  (void)::write(lock_fd_, owner.data(), owner.size());
}

void SerialPort::releaseLock() {
  if (lock_fd_ >= 0) {
    (void)::flock(lock_fd_, LOCK_UN);
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
  lock_path_.clear();
}

const std::string& SerialPort::lockPath() const noexcept { return lock_path_; }

bool SerialPort::isOpen() const noexcept { return fd_ >= 0; }
int SerialPort::fd() const noexcept { return fd_; }

bool SerialPort::waitFor(
    const short events,
    const std::chrono::steady_clock::time_point deadline,
    std::string& error) {
  while (true) {
    const auto remaining = remainingMs(deadline);
    if (remaining.count() <= 0) {
      error = "timeout";
      return false;
    }

    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = events;

    const int rc = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (rc > 0) {
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        error = "poll reported UART error/hangup";
        return false;
      }
      if ((descriptor.revents & events) != 0) {
        return true;
      }
      continue;
    }
    if (rc == 0) {
      error = "timeout";
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    error = std::string("poll failed: ") + std::strerror(errno);
    return false;
  }
}

IoStatus SerialPort::writeAll(
    const std::vector<std::uint8_t>& data,
    const std::chrono::milliseconds timeout,
    std::string& error) {
  if (!isOpen()) {
    error = "UART is not open";
    return IoStatus::kIoError;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t written = 0;

  while (written < data.size()) {
    const ssize_t rc = ::write(
        fd_, data.data() + written, data.size() - written);
    if (rc > 0) {
      written += static_cast<std::size_t>(rc);
      continue;
    }
    if (rc < 0 && errno == EINTR) {
      continue;
    }
    if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (!waitFor(POLLOUT, deadline, error)) {
        return error == "timeout" ? IoStatus::kTimeout : IoStatus::kIoError;
      }
      continue;
    }

    error = std::string("write failed: ") + std::strerror(errno);
    return IoStatus::kIoError;
  }

  error.clear();
  return IoStatus::kOk;
}

bool SerialPort::extractFrame(std::vector<std::uint8_t>& frame, std::string& error) {
  while (rx_buffer_.size() >= 2U) {
    if (rx_buffer_[0] == kHeader && rx_buffer_[1] == kHeader) {
      break;
    }
    rx_buffer_.erase(rx_buffer_.begin());
  }

  if (rx_buffer_.size() < 4U) {
    return false;
  }

  const std::size_t length = rx_buffer_[3];
  const std::size_t frame_size = 4U + length;
  if (frame_size < 6U || frame_size > 260U) {
    error = "invalid ST3215 frame length";
    rx_buffer_.erase(rx_buffer_.begin());
    return false;
  }

  if (rx_buffer_.size() < frame_size) {
    return false;
  }

  frame.assign(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  return true;
}

FrameReadResult SerialPort::readFrame(
    const std::uint8_t expected_id,
    const std::chrono::milliseconds timeout) {
  FrameReadResult result;
  if (!isOpen()) {
    result.status = IoStatus::kIoError;
    result.message = "UART is not open";
    return result;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<std::uint8_t> candidate;
    std::string parse_error;
    if (extractFrame(candidate, parse_error)) {
      if (!validateFrameChecksum(candidate)) {
        result.status = IoStatus::kChecksumError;
        result.message = "reply checksum mismatch";
        result.frame = std::move(candidate);
        return result;
      }
      if (candidate.size() < 3U || candidate[2] != expected_id) {
        result.status = IoStatus::kWrongServoId;
        result.message = "reply servo ID does not match request";
        result.frame = std::move(candidate);
        return result;
      }
      result.status = IoStatus::kOk;
      result.frame = std::move(candidate);
      return result;
    }
    if (!parse_error.empty()) {
      result.status = IoStatus::kMalformedFrame;
      result.message = parse_error;
      return result;
    }

    std::string wait_error;
    if (!waitFor(POLLIN, deadline, wait_error)) {
      result.status = wait_error == "timeout" ? IoStatus::kTimeout : IoStatus::kIoError;
      result.message = wait_error;
      return result;
    }

    std::uint8_t temp[256];
    while (true) {
      const ssize_t rc = ::read(fd_, temp, sizeof(temp));
      if (rc > 0) {
        rx_buffer_.insert(rx_buffer_.end(), temp, temp + rc);
        if (static_cast<std::size_t>(rc) < sizeof(temp)) {
          break;
        }
        continue;
      }
      if (rc < 0 && errno == EINTR) {
        continue;
      }
      if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (rc == 0) {
        break;
      }
      result.status = IoStatus::kIoError;
      result.message = std::string("read failed: ") + std::strerror(errno);
      return result;
    }
  }

  result.status = IoStatus::kTimeout;
  result.message = "timeout";
  return result;
}

void SerialPort::flushInput() {
  rx_buffer_.clear();
  if (isOpen()) {
    ::tcflush(fd_, TCIFLUSH);
  }
}

const char* toString(const IoStatus status) noexcept {
  switch (status) {
    case IoStatus::kOk:
      return "ok";
    case IoStatus::kTimeout:
      return "timeout";
    case IoStatus::kChecksumError:
      return "checksum_error";
    case IoStatus::kMalformedFrame:
      return "malformed_frame";
    case IoStatus::kWrongServoId:
      return "wrong_servo_id";
    case IoStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

}  // namespace lgh_st3215_driver
