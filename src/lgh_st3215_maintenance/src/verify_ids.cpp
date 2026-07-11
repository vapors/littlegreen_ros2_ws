#include "lgh_st3215_maintenance/common.hpp"
#include "lgh_st3215_driver/joint_map.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <set>
#include <string>

using namespace lgh_st3215_driver;
using namespace lgh_st3215_maintenance;

int main(int argc, char** argv) {
  std::string port_name = "/dev/ttyS3";
  std::string map_path = ament_index_cpp::get_package_share_directory("lgh_st3215_driver") + "/config/servo_map.yaml";
  int baud = 1000000, timeout_ms = 5;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto next = [&]() -> std::string { if (++i >= argc) throw std::invalid_argument("missing value for " + arg); return argv[i]; };
      if (arg == "--port") port_name = next();
      else if (arg == "--baud") baud = parseInt(next());
      else if (arg == "--servo-map") map_path = next();
      else if (arg == "--timeout-ms") timeout_ms = parseInt(next());
      else if (arg == "--help") { std::cout << "verify_ids [--servo-map path] [--port /dev/ttyS3]\n"; return kPass; }
      else throw std::invalid_argument("unknown argument: " + arg);
    }
    const JointMap map = JointMap::loadFromYaml(map_path, 0, 0);
    std::set<int> unique;
    for (const auto& joint : map.joints()) {
      if (!unique.insert(joint.servo_id).second) { std::cerr << "Duplicate ID in map: " << static_cast<int>(joint.servo_id) << "\n"; return kConfig; }
    }
    SerialPort serial; serial.openPort(port_name, baud);
    bool all_ok = true;
    for (const auto& joint : map.joints()) {
      FrameReadResult reply; std::string error;
      const bool replied = transact(serial, buildPingRequest(joint.servo_id), joint.servo_id, 5, timeout_ms, reply, error);
      const int status_error = replied && reply.frame.size() >= 6U ? static_cast<int>(reply.frame[4]) : -1;
      const bool ok = replied && status_error == 0;
      std::cout << (ok ? "PASS " : "FAIL ") << joint.name << " id=" << static_cast<int>(joint.servo_id);
      if (!replied) std::cout << " error=" << error;
      else if (status_error != 0) std::cout << " status_error=" << status_error;
      std::cout << "\n";
      all_ok = all_ok && ok;
    }
    std::cout << "ST3215 ID VERIFICATION: " << (all_ok ? "PASS" : "FAIL") << "\n";
    return all_ok ? kPass : kTestFail;
  } catch (const std::invalid_argument& error) { std::cerr << "CONFIG ERROR: " << error.what() << "\n"; return kConfig; }
    catch (const std::exception& error) { std::cerr << "ST3215 ID VERIFICATION REFUSED/FAILED: " << error.what() << "\n"; return exceptionExit(error); }
}
