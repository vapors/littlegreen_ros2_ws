#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lgh_st3215_driver {

constexpr std::size_t kNumJoints = 12;
constexpr double kStepsPerRevolution = 4096.0;
constexpr double kStepsPerRadian = kStepsPerRevolution / (2.0 * 3.14159265358979323846);
constexpr double kRadiansPerStep = (2.0 * 3.14159265358979323846) / kStepsPerRevolution;

struct JointConfig {
  std::string name;
  std::size_t policy_index{0};
  std::uint8_t servo_id{0};
  int servo_sign{1};
  double joint_zero_rad{0.0};
  double training_default_rad{0.0};
  int center_step{2048};
  double min_rad{-3.14159265358979323846};
  double max_rad{3.14159265358979323846};
  int min_step{0};
  int max_step{4095};
  std::uint16_t speed{0};
  std::uint8_t acceleration{0};
};

class JointMap {
 public:
  static JointMap loadFromYaml(
      const std::string& path,
      std::uint16_t default_speed,
      std::uint8_t default_acceleration);

  const std::vector<JointConfig>& joints() const noexcept;
  const JointConfig& at(std::size_t index) const;
  std::size_t size() const noexcept;

  int radiansToSteps(std::size_t joint_index, double target_rad) const;
  double stepsToRadians(std::size_t joint_index, int raw_step) const;

 private:
  std::vector<JointConfig> joints_;
};

}  // namespace lgh_st3215_driver
