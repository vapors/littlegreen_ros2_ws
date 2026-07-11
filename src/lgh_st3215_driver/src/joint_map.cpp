#include "lgh_st3215_driver/joint_map.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace lgh_st3215_driver {
namespace {

template <typename T>
T valueOr(const YAML::Node& node, const char* key, const T& fallback) {
  if (node[key] && !node[key].IsNull()) {
    return node[key].as<T>();
  }
  return fallback;
}

template <typename T>
T valueOrEither(
    const YAML::Node& node,
    const char* primary,
    const char* alternate,
    const T& fallback) {
  if (node[primary] && !node[primary].IsNull()) {
    return node[primary].as<T>();
  }
  if (node[alternate] && !node[alternate].IsNull()) {
    return node[alternate].as<T>();
  }
  return fallback;
}

}  // namespace

JointMap JointMap::loadFromYaml(
    const std::string& path,
    const std::uint16_t default_speed,
    const std::uint8_t default_acceleration) {
  YAML::Node root = YAML::LoadFile(path);
  if (!root["joints"] || !root["joints"].IsSequence()) {
    throw std::runtime_error("Joint map YAML does not contain a joints sequence: " + path);
  }

  std::vector<JointConfig> joints;
  joints.reserve(root["joints"].size());

  for (const auto& node : root["joints"]) {
    JointConfig joint;
    joint.name = node["name"].as<std::string>();
    joint.policy_index = valueOrEither<std::size_t>(
        node, "policy_index", "policy_action_index", joints.size());
    const int servo_id = node["servo_id"].as<int>();
    if (servo_id < 0 || servo_id > 253) {
      throw std::runtime_error("servo_id must be in 0..253 for " + joint.name);
    }
    joint.servo_id = static_cast<std::uint8_t>(servo_id);
    joint.servo_sign = valueOr<int>(node, "servo_sign", 1);
    joint.joint_zero_rad = valueOr<double>(node, "joint_zero_rad", 0.0);
    joint.training_default_rad = valueOrEither<double>(
        node, "training_default_rad", "default_joint_rad", 0.0);
    joint.center_step = valueOrEither<int>(node, "center_step", "servo_center_step", 2048);
    joint.min_rad = valueOrEither<double>(node, "min_rad", "limit_lower_rad", -3.14159265358979323846);
    joint.max_rad = valueOrEither<double>(node, "max_rad", "limit_upper_rad", 3.14159265358979323846);
    joint.min_step = valueOrEither<int>(node, "min_step", "servo_min_step", 0);
    joint.max_step = valueOrEither<int>(node, "max_step", "servo_max_step", 4095);
    const int speed = valueOr<int>(node, "speed", static_cast<int>(default_speed));
    if (speed < 0 || speed > 32767) {
      throw std::runtime_error("speed must be in 0..32767 for " + joint.name);
    }
    joint.speed = static_cast<std::uint16_t>(speed);
    const int acceleration = valueOr<int>(
        node, "acceleration", static_cast<int>(default_acceleration));
    if (acceleration < 0 || acceleration > 254) {
      throw std::runtime_error("acceleration must be in 0..254 for " + joint.name);
    }
    joint.acceleration = static_cast<std::uint8_t>(acceleration);

    if (joint.servo_sign != -1 && joint.servo_sign != 1) {
      throw std::runtime_error("servo_sign must be +1 or -1 for " + joint.name);
    }
    if (joint.min_rad > joint.max_rad) {
      throw std::runtime_error("min_rad > max_rad for " + joint.name);
    }
    if (joint.training_default_rad < joint.min_rad ||
        joint.training_default_rad > joint.max_rad) {
      throw std::runtime_error(
          "training_default_rad lies outside joint limits for " + joint.name);
    }
    if (joint.min_step > joint.max_step) {
      throw std::runtime_error("min_step > max_step for " + joint.name);
    }
    joints.push_back(joint);
  }

  if (joints.size() != kNumJoints) {
    throw std::runtime_error(
        "Native single-bus driver currently requires exactly 12 joints; YAML contains " +
        std::to_string(joints.size()));
  }

  std::sort(
      joints.begin(), joints.end(),
      [](const JointConfig& a, const JointConfig& b) {
        return a.policy_index < b.policy_index;
      });

  std::set<std::uint8_t> servo_ids;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].policy_index != i) {
      throw std::runtime_error(
          "policy indices must be contiguous 0..11 in native servo map");
    }
    if (!servo_ids.insert(joints[i].servo_id).second) {
      throw std::runtime_error(
          "duplicate servo ID " + std::to_string(joints[i].servo_id));
    }
  }

  JointMap map;
  map.joints_ = std::move(joints);
  return map;
}

const std::vector<JointConfig>& JointMap::joints() const noexcept { return joints_; }

const JointConfig& JointMap::at(const std::size_t index) const {
  return joints_.at(index);
}

std::size_t JointMap::size() const noexcept { return joints_.size(); }

int JointMap::radiansToSteps(const std::size_t joint_index, const double target_rad) const {
  const auto& joint = at(joint_index);
  const double clamped_rad = std::clamp(target_rad, joint.min_rad, joint.max_rad);
  const double step_value =
      static_cast<double>(joint.center_step) +
      static_cast<double>(joint.servo_sign) *
          (clamped_rad - joint.joint_zero_rad) * kStepsPerRadian;
  const int rounded = static_cast<int>(std::llround(step_value));
  return std::clamp(rounded, joint.min_step, joint.max_step);
}

double JointMap::stepsToRadians(const std::size_t joint_index, const int raw_step) const {
  const auto& joint = at(joint_index);
  const double delta_steps = static_cast<double>(raw_step - joint.center_step);
  return joint.joint_zero_rad +
         (delta_steps / static_cast<double>(joint.servo_sign)) * kRadiansPerStep;
}

}  // namespace lgh_st3215_driver
