#include "lgh_st3215_maintenance/common.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace lgh_st3215_driver;
using namespace lgh_st3215_maintenance;

int main(int argc, char** argv) {
  std::string port_name = "/dev/ttyS3";
  std::string output;
  int baud = 1000000, first_id = 1, last_id = 253, timeout_ms = 3;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto next = [&]() -> std::string { if (++i >= argc) throw std::invalid_argument("missing value for " + arg); return argv[i]; };
      if (arg == "--port") port_name = next();
      else if (arg == "--baud") baud = parseInt(next());
      else if (arg == "--first-id") first_id = parseInt(next());
      else if (arg == "--last-id") last_id = parseInt(next());
      else if (arg == "--timeout-ms") timeout_ms = parseInt(next());
      else if (arg == "--output") output = next();
      else if (arg == "--help") { std::cout << "bus_scan [--port /dev/ttyS3] [--first-id 1] [--last-id 253] [--timeout-ms 3] [--output scan.yaml]\n"; return kPass; }
      else throw std::invalid_argument("unknown argument: " + arg);
    }
    if (first_id < 0 || last_id > 253 || first_id > last_id || timeout_ms <= 0) return kConfig;
    SerialPort serial;
    serial.openPort(port_name, baud);
    std::vector<int> found;
    for (int id = first_id; id <= last_id; ++id) {
      FrameReadResult reply; std::string error;
      if (transact(serial, buildPingRequest(static_cast<std::uint8_t>(id)), static_cast<std::uint8_t>(id), 5, timeout_ms, reply, error)) {
        found.push_back(id);
        std::cout << "FOUND id=" << id << " status_error=" << static_cast<unsigned int>(reply.frame[4]) << "\n";
      }
    }
    std::cout << "ST3215 BUS SCAN: " << (found.empty() ? "FAIL" : "PASS") << " found=" << found.size() << "\n";
    if (!output.empty()) {
      std::ofstream file(output);
      file << "schema_version: 1\nport: " << port_name << "\nbaud: " << baud << "\nfound_ids: [";
      for (std::size_t i = 0; i < found.size(); ++i) { if (i) file << ", "; file << found[i]; }
      file << "]\n";
    }
    return found.empty() ? kTestFail : kPass;
  } catch (const std::invalid_argument& error) {
    std::cerr << "CONFIG ERROR: " << error.what() << "\n"; return kConfig;
  } catch (const std::exception& error) {
    std::cerr << "ST3215 BUS SCAN REFUSED/FAILED: " << error.what() << "\n"; return exceptionExit(error);
  }
}
