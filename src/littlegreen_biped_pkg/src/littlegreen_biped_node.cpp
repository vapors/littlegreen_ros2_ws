// littlegreen_biped_node.cpp
// LittleGreen ROS 2 policy inference node.
//
// Responsibilities:
//   - Load a paired deployment YAML + ONNX policy artifact.
//   - Validate action-contract-v3/v4 defaults, bounds, residual scales, and action mapping against joint_map.yaml.
//   - Build either the legacy 45-D observation or the phase-guided 47-D observation.
//   - Reproduce the deterministic Track 1 gait phase clock for 47-D policies.
//   - Apply the physical IMU-frame -> base-frame extrinsic transform.
//   - Gate inference on complete, fresh, finite sensor data.
//   - Reject non-finite observations and ONNX outputs.
//   - Publish 12 target joint positions in radians on /desired_position.
//
// This node intentionally does not implement servo-bus conversion or actuator-level control.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include "littlegreen_biped_pkg/policy_observation_contract.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{
using SteadyClock = std::chrono::steady_clock;
using littlegreen_biped::GaitPhaseClock;
using littlegreen_biped::GaitPhaseSample;

bool is_finite(float value)
{
    return std::isfinite(static_cast<double>(value));
}

bool is_finite(double value)
{
    return std::isfinite(value);
}

template <typename ContainerT>
bool all_finite(const ContainerT& values)
{
    return std::all_of(values.begin(), values.end(), [](const auto& value) {
        return std::isfinite(static_cast<double>(value));
    });
}

std::string parent_directory(const std::string& path)
{
    const auto slash = path.find_last_of("/\\");
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

class Sha256
{
public:
    Sha256()
    : state_{
          0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
          0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}
    {
    }

    void update(const uint8_t* input, size_t length)
    {
        for (size_t i = 0; i < length; ++i) {
            block_[block_length_++] = input[i];
            if (block_length_ == 64) {
                transform(block_.data());
                bit_length_ += 512;
                block_length_ = 0;
            }
        }
    }

    std::array<uint8_t, 32> final()
    {
        size_t i = block_length_;

        block_[i++] = 0x80U;
        if (i > 56) {
            while (i < 64) {
                block_[i++] = 0x00U;
            }
            transform(block_.data());
            i = 0;
        }
        while (i < 56) {
            block_[i++] = 0x00U;
        }

        bit_length_ += static_cast<uint64_t>(block_length_) * 8U;
        for (int shift = 56, index = 56; shift >= 0; shift -= 8, ++index) {
            block_[static_cast<size_t>(index)] =
                static_cast<uint8_t>((bit_length_ >> shift) & 0xffU);
        }
        transform(block_.data());

        std::array<uint8_t, 32> digest{};
        for (size_t word = 0; word < 8; ++word) {
            digest[word * 4 + 0] = static_cast<uint8_t>((state_[word] >> 24) & 0xffU);
            digest[word * 4 + 1] = static_cast<uint8_t>((state_[word] >> 16) & 0xffU);
            digest[word * 4 + 2] = static_cast<uint8_t>((state_[word] >> 8) & 0xffU);
            digest[word * 4 + 3] = static_cast<uint8_t>(state_[word] & 0xffU);
        }
        return digest;
    }

private:
    static uint32_t rotate_right(uint32_t value, uint32_t count)
    {
        return (value >> count) | (value << (32U - count));
    }

    static uint32_t choose(uint32_t x, uint32_t y, uint32_t z)
    {
        return (x & y) ^ (~x & z);
    }

    static uint32_t majority(uint32_t x, uint32_t y, uint32_t z)
    {
        return (x & y) ^ (x & z) ^ (y & z);
    }

    static uint32_t big_sigma0(uint32_t x)
    {
        return rotate_right(x, 2) ^ rotate_right(x, 13) ^ rotate_right(x, 22);
    }

    static uint32_t big_sigma1(uint32_t x)
    {
        return rotate_right(x, 6) ^ rotate_right(x, 11) ^ rotate_right(x, 25);
    }

    static uint32_t small_sigma0(uint32_t x)
    {
        return rotate_right(x, 7) ^ rotate_right(x, 18) ^ (x >> 3);
    }

    static uint32_t small_sigma1(uint32_t x)
    {
        return rotate_right(x, 17) ^ rotate_right(x, 19) ^ (x >> 10);
    }

    void transform(const uint8_t* block)
    {
        static constexpr std::array<uint32_t, 64> k{
            0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
            0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
            0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
            0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
            0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
            0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
            0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
            0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
            0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
            0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
            0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
            0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
            0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
            0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
            0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
            0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

        std::array<uint32_t, 64> schedule{};
        for (size_t i = 0; i < 16; ++i) {
            const size_t offset = i * 4;
            schedule[i] =
                (static_cast<uint32_t>(block[offset]) << 24) |
                (static_cast<uint32_t>(block[offset + 1]) << 16) |
                (static_cast<uint32_t>(block[offset + 2]) << 8) |
                static_cast<uint32_t>(block[offset + 3]);
        }
        for (size_t i = 16; i < 64; ++i) {
            schedule[i] = small_sigma1(schedule[i - 2]) + schedule[i - 7] +
                          small_sigma0(schedule[i - 15]) + schedule[i - 16];
        }

        uint32_t a = state_[0];
        uint32_t b = state_[1];
        uint32_t c = state_[2];
        uint32_t d = state_[3];
        uint32_t e = state_[4];
        uint32_t f = state_[5];
        uint32_t g = state_[6];
        uint32_t h = state_[7];

        for (size_t i = 0; i < 64; ++i) {
            const uint32_t temp1 = h + big_sigma1(e) + choose(e, f, g) + k[i] + schedule[i];
            const uint32_t temp2 = big_sigma0(a) + majority(a, b, c);
            h = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }

        state_[0] += a;
        state_[1] += b;
        state_[2] += c;
        state_[3] += d;
        state_[4] += e;
        state_[5] += f;
        state_[6] += g;
        state_[7] += h;
    }

    std::array<uint32_t, 8> state_{};
    std::array<uint8_t, 64> block_{};
    size_t block_length_ = 0;
    uint64_t bit_length_ = 0;
};

std::string sha256_file(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("unable to open file for SHA-256: " + path);
    }

    Sha256 sha256;
    std::array<char, 8192> buffer{};
    while (file.good()) {
        file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto count = file.gcount();
        if (count > 0) {
            sha256.update(
                reinterpret_cast<const uint8_t*>(buffer.data()),
                static_cast<size_t>(count));
        }
    }

    const auto digest = sha256.final();
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const uint8_t byte : digest) {
        stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
}
}  // namespace

class LittleGreenBipedPolicyNode : public rclcpp::Node
{
public:
    LittleGreenBipedPolicyNode()
    : Node("littlegreen_biped_node"),
      env_(ORT_LOGGING_LEVEL_WARNING, "littlegreen_biped_policy")
    {
        const std::string package_share_dir =
            ament_index_cpp::get_package_share_directory("littlegreen_biped_pkg");

        declare_parameters(package_share_dir);
        read_runtime_parameters();

        std::string policy_config_path;
        std::string joint_map_path;
        std::string onnx_model_path_override;
        this->get_parameter("policy_config_path", policy_config_path);
        this->get_parameter("joint_map_path", joint_map_path);
        this->get_parameter("onnx_model_path", onnx_model_path_override);

        load_policy_config(policy_config_path);
        load_joint_map(joint_map_path);
        validate_policy_contract();
        initialize_state_storage();

        const std::string model_path = resolve_model_path(policy_config_path, onnx_model_path_override);
        load_onnx_model(model_path);

        const auto sensor_best_effort_qos = rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort();
        const auto sensor_reliable_qos = rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliable();

        if (policy_output_mode_ == "live") {
            desired_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/desired_position", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        } else if (policy_output_mode_ == "shadow") {
            policy_shadow_desired_position_pub_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    shadow_desired_position_topic_,
                    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
        }

        policy_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/policy_ready", rclcpp::QoS(1).reliable().transient_local());
        policy_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/policy_status", rclcpp::QoS(1).reliable().transient_local());

        if (publish_policy_debug_) {
            const auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
            policy_debug_observation_pub_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/policy_debug/observation", debug_qos);
            policy_debug_raw_action_pub_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/policy_debug/raw_action", debug_qos);
            policy_debug_clipped_raw_action_pub_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/policy_debug/clipped_raw_action", debug_qos);
            policy_debug_target_unclipped_pub_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/policy_debug/target_unclipped", debug_qos);
            policy_debug_target_clipped_pub_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/policy_debug/target_clipped", debug_qos);
            policy_debug_saturation_mask_pub_ =
                this->create_publisher<std_msgs::msg::UInt8MultiArray>(
                    "/policy_debug/saturation_mask", debug_qos);
            if (gait_phase_enabled_) {
                policy_debug_gait_phase_pub_ =
                    this->create_publisher<std_msgs::msg::Float64MultiArray>(
                        "/policy_debug/gait_phase", debug_qos);
            }
        }

        if (gait_phase_enabled_) {
            reset_gait_phase_service_ = this->create_service<std_srvs::srv::Trigger>(
                "/policy/reset_gait_phase",
                std::bind(
                    &LittleGreenBipedPolicyNode::reset_gait_phase_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

        if (override_imu_) {
            RCLCPP_WARN(
                this->get_logger(),
                "IMU override enabled: policy will use angular_velocity=[0,0,0] and projected_gravity=[0,0,-1].");
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribing to /imu/data and applying configured IMU->base extrinsic transform.");
            imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", use_sim_ ? sensor_reliable_qos : sensor_best_effort_qos,
                std::bind(&LittleGreenBipedPolicyNode::imu_callback, this, std::placeholders::_1));
        }

        joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", use_sim_ ? sensor_reliable_qos : sensor_best_effort_qos,
            std::bind(&LittleGreenBipedPolicyNode::joint_state_callback, this, std::placeholders::_1));

        if (!use_sim_ && require_joint_feedback_age_) {
            joint_feedback_age_subscriber_ =
                this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                    "/joint_feedback_age_ms", sensor_best_effort_qos,
                    std::bind(
                        &LittleGreenBipedPolicyNode::joint_feedback_age_callback,
                        this,
                        std::placeholders::_1));
        }

        // Backward compatibility only. These split topics are not required by the current servo firmware.
        if (!use_sim_) {
            joint_position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states_position", sensor_best_effort_qos,
                std::bind(&LittleGreenBipedPolicyNode::joint_state_callback, this, std::placeholders::_1));
            joint_velocity_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states_velocity", sensor_best_effort_qos,
                std::bind(&LittleGreenBipedPolicyNode::joint_state_callback, this, std::placeholders::_1));
        }

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/command_velocity", 10,
            std::bind(&LittleGreenBipedPolicyNode::cmd_vel_callback, this, std::placeholders::_1));

        // Retained so existing launch graphs that expect /joy connectivity continue to work.
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&LittleGreenBipedPolicyNode::joy_callback, this, std::placeholders::_1));

        const auto policy_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(policy_dt_));
        timer_ = this->create_wall_timer(
            policy_period,
            std::bind(&LittleGreenBipedPolicyNode::run_policy_and_publish, this));

        publish_policy_status(
            false,
            use_sim_ || !require_joint_feedback_age_
                ? "waiting for fresh IMU and complete joint state"
                : "waiting for fresh IMU, complete joint state, and hardware feedback ages");

        RCLCPP_INFO(
            this->get_logger(),
            "LittleGreen policy node initialized: obs[%zu] -> actions[%zu], observation_contract=v%d/%s, policy_dt=%.3fs, output_mode=%s.",
            num_observations_, num_actions_, observation_contract_version_,
            observation_contract_name_.c_str(), policy_dt_, policy_output_mode_.c_str());
        if (policy_output_mode_ == "shadow") {
            RCLCPP_WARN(
                this->get_logger(),
                "POLICY OUTPUT MODE: SHADOW. /desired_position will not be published; proposed targets use %s.",
                shadow_desired_position_topic_.c_str());
        } else if (policy_output_mode_ == "disabled") {
            RCLCPP_WARN(
                this->get_logger(),
                "POLICY OUTPUT MODE: DISABLED. Sensor readiness is evaluated but ONNX inference and target publication are disabled.");
        }
        if (gait_phase_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "Gait phase lifecycle: starts at [sin,cos]=[0,1], advances only after successful inference, freezes while readiness is gated, and resets on node restart.");
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Freshness limits: imu=%.3fs, joint transport=%.3fs, feedback-age topic=%.3fs, "
            "hardware feedback=%.3fs, command timeout=%.3fs.",
            imu_timeout_sec_,
            joint_state_timeout_sec_,
            joint_feedback_age_topic_timeout_sec_,
            joint_feedback_max_age_sec_,
            command_timeout_sec_);
        RCLCPP_INFO(
            this->get_logger(),
            "IMU->base matrix row-major: [%.1f %.1f %.1f; %.1f %.1f %.1f; %.1f %.1f %.1f]",
            imu_to_base_matrix_[0], imu_to_base_matrix_[1], imu_to_base_matrix_[2],
            imu_to_base_matrix_[3], imu_to_base_matrix_[4], imu_to_base_matrix_[5],
            imu_to_base_matrix_[6], imu_to_base_matrix_[7], imu_to_base_matrix_[8]);
    }

private:
    struct JointMapEntry
    {
        std::string name;
        int policy_action_index = -1;
        int sim_joint_index = -1;
        int ros_joint_state_index = -1;
        int micro_ros_array_index = -1;
        int servo_id = -1;
        float default_joint_rad = 0.0f;
        float limit_lower_rad = 0.0f;
        float limit_upper_rad = 0.0f;
    };

    struct PolicyStateSnapshot
    {
        std::vector<float> cmd_vel;
        std::vector<float> base_ang_vel;
        std::array<float, 4> imu_orientation_wxyz{1.0f, 0.0f, 0.0f, 0.0f};
        std::vector<float> joint_positions;
        std::vector<float> joint_velocities;
        std::vector<float> prev_actions;
    };

    struct PhaseSnapshot
    {
        GaitPhaseSample sample;
        std::uint64_t generation = 0U;
    };

    void declare_parameters(const std::string& package_share_dir)
    {
        this->declare_parameter<bool>("use_sim", false);
        this->declare_parameter<bool>("override_imu", false);
        this->declare_parameter<std::string>(
            "policy_config_path", package_share_dir + "/configs/policy_latest.yaml");
        this->declare_parameter<std::string>(
            "joint_map_path", package_share_dir + "/configs/joint_map.yaml");
        this->declare_parameter<std::string>("onnx_model_path", "");
        this->declare_parameter<bool>("publish_policy_debug", true);
        this->declare_parameter<std::string>("policy_output_mode", "live");
        this->declare_parameter<std::string>(
            "shadow_desired_position_topic", "/policy_shadow/desired_position");

        this->declare_parameter<double>("imu_timeout_sec", 0.050);
        // Transport freshness for the cached /joint_states message stream.
        this->declare_parameter<double>("joint_state_timeout_sec", 0.150);

        // Physical feedback freshness comes from /joint_feedback_age_ms. The
        // topic timeout checks host transport freshness; max age checks the true
        // age of each servo's last successful bus read.
        this->declare_parameter<bool>("require_joint_feedback_age", true);
        this->declare_parameter<double>("joint_feedback_age_topic_timeout_sec", 0.150);
        this->declare_parameter<double>("joint_feedback_max_age_sec", 0.250);

        this->declare_parameter<double>("command_timeout_sec", 0.500);
        this->declare_parameter<bool>("require_joint_velocity", true);
        this->declare_parameter<bool>("zero_command_on_timeout", true);

        // Row-major transform for vectors expressed in the physical IMU frame:
        //   x_base =  y_imu
        //   y_base = -x_imu
        //   z_base =  z_imu
        this->declare_parameter<std::vector<double>>(
            "imu_to_base_matrix",
            std::vector<double>{
                0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0,
                0.0, 0.0, 1.0});
    }

    void read_runtime_parameters()
    {
        this->get_parameter("use_sim", use_sim_);
        this->get_parameter("override_imu", override_imu_);
        this->get_parameter("publish_policy_debug", publish_policy_debug_);
        this->get_parameter("policy_output_mode", policy_output_mode_);
        this->get_parameter("shadow_desired_position_topic", shadow_desired_position_topic_);
        if (policy_output_mode_ != "live" &&
            policy_output_mode_ != "shadow" &&
            policy_output_mode_ != "disabled") {
            throw std::runtime_error(
                "policy_output_mode must be one of: live, shadow, disabled");
        }
        this->get_parameter("imu_timeout_sec", imu_timeout_sec_);
        this->get_parameter("joint_state_timeout_sec", joint_state_timeout_sec_);
        this->get_parameter("require_joint_feedback_age", require_joint_feedback_age_);
        this->get_parameter(
            "joint_feedback_age_topic_timeout_sec",
            joint_feedback_age_topic_timeout_sec_);
        this->get_parameter("joint_feedback_max_age_sec", joint_feedback_max_age_sec_);
        this->get_parameter("command_timeout_sec", command_timeout_sec_);
        this->get_parameter("require_joint_velocity", require_joint_velocity_);
        this->get_parameter("zero_command_on_timeout", zero_command_on_timeout_);

        std::vector<double> matrix;
        this->get_parameter("imu_to_base_matrix", matrix);
        if (matrix.size() != 9 || !all_finite(matrix)) {
            throw std::runtime_error("imu_to_base_matrix must contain exactly 9 finite values");
        }
        for (size_t i = 0; i < 9; ++i) {
            imu_to_base_matrix_[i] = static_cast<float>(matrix[i]);
        }

        if (!(imu_timeout_sec_ > 0.0) ||
            !(joint_state_timeout_sec_ > 0.0) ||
            !(joint_feedback_age_topic_timeout_sec_ > 0.0) ||
            !(joint_feedback_max_age_sec_ > 0.0) ||
            !(command_timeout_sec_ > 0.0)) {
            throw std::runtime_error("freshness timeout parameters must be positive");
        }
    }

    static bool file_exists(const std::string& path)
    {
        std::ifstream file(path, std::ios::binary);
        return file.good();
    }

    static float clamp_float(float value, float lower, float upper)
    {
        // Callers must already reject non-finite values.
        return std::max(lower, std::min(upper, value));
    }

    static std::vector<float> load_float_vector(
        const YAML::Node& node,
        size_t expected_size,
        const std::string& field_name)
    {
        if (!node) {
            throw std::runtime_error("Missing YAML field: " + field_name);
        }

        std::vector<float> values;
        if (node.IsScalar()) {
            values.assign(expected_size, node.as<float>());
        } else if (node.IsSequence()) {
            values.reserve(node.size());
            for (const auto& value : node) {
                values.push_back(value.as<float>());
            }
        } else {
            throw std::runtime_error("YAML field is not scalar or sequence: " + field_name);
        }

        if (values.size() != expected_size) {
            throw std::runtime_error(
                "YAML field size mismatch for " + field_name +
                ": expected " + std::to_string(expected_size) +
                ", got " + std::to_string(values.size()));
        }
        if (!all_finite(values)) {
            throw std::runtime_error("YAML field contains a non-finite value: " + field_name);
        }
        return values;
    }

    static std::vector<int> load_int_vector(
        const YAML::Node& node,
        size_t expected_size,
        const std::string& field_name)
    {
        if (!node || !node.IsSequence()) {
            throw std::runtime_error("Missing or non-sequence YAML field: " + field_name);
        }

        std::vector<int> values;
        values.reserve(node.size());
        for (const auto& value : node) {
            values.push_back(value.as<int>());
        }
        if (values.size() != expected_size) {
            throw std::runtime_error(
                "YAML field size mismatch for " + field_name +
                ": expected " + std::to_string(expected_size) +
                ", got " + std::to_string(values.size()));
        }
        return values;
    }

    static std::vector<std::string> load_string_vector(
        const YAML::Node& node,
        size_t expected_size,
        const std::string& field_name)
    {
        if (!node || !node.IsSequence()) {
            throw std::runtime_error("Missing or non-sequence YAML field: " + field_name);
        }

        std::vector<std::string> values;
        values.reserve(node.size());
        for (const auto& value : node) {
            values.push_back(value.as<std::string>());
        }
        if (values.size() != expected_size) {
            throw std::runtime_error(
                "YAML field size mismatch for " + field_name +
                ": expected " + std::to_string(expected_size) +
                ", got " + std::to_string(values.size()));
        }
        return values;
    }

    void load_observation_contract()
    {
        using littlegreen_biped::kLegacyObservationCount;

        if (!littlegreen_biped::is_supported_observation_count(num_observations_)) {
            throw std::runtime_error(
                "Unsupported policy observation count " + std::to_string(num_observations_) +
                ". Supported deployment contracts are 45-D legacy and 47-D phase-guided.");
        }

        observation_contract_version_ = policy_config_["observation_contract_version"]
            ? policy_config_["observation_contract_version"].as<int>()
            : (num_observations_ == kLegacyObservationCount ? 1 : 0);
        observation_contract_name_ = policy_config_["observation_contract_name"]
            ? policy_config_["observation_contract_name"].as<std::string>()
            : (num_observations_ == kLegacyObservationCount
                ? "littlegreen_hardware_45_legacy"
                : std::string{});

        if (num_observations_ == kLegacyObservationCount) {
            if (observation_contract_version_ != 1) {
                throw std::runtime_error(
                    "45-D policy requires observation_contract_version 1 when the field is present");
            }
            if (policy_config_["observation_contract_name"] &&
                observation_contract_name_ != "littlegreen_hardware_45_v1" &&
                observation_contract_name_ != "littlegreen_hardware_45_legacy") {
                throw std::runtime_error(
                    "45-D observation_contract_name must be littlegreen_hardware_45_v1 "
                    "or littlegreen_hardware_45_legacy");
            }
            if (policy_config_["gait_phase_enabled"] &&
                policy_config_["gait_phase_enabled"].as<bool>()) {
                throw std::runtime_error("45-D policy cannot enable gait-phase observations");
            }
            gait_phase_enabled_ = false;
            if (!policy_config_["observation_contract_version"] ||
                !policy_config_["observation_contract_name"]) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Legacy 45-D policy bundle has no explicit observation-contract metadata. "
                    "Accepted for legacy compatibility; new exports should declare contract v1.");
            }
            return;
        }

        if (observation_contract_version_ != 2) {
            throw std::runtime_error(
                "47-D policy requires observation_contract_version: 2");
        }
        if (observation_contract_name_ != "littlegreen_hardware_phase_guided_47_v1") {
            throw std::runtime_error(
                "47-D policy requires observation_contract_name: "
                "littlegreen_hardware_phase_guided_47_v1");
        }

        const std::vector<std::string> expected_layout{
            "command_velocity_3",
            "base_angular_velocity_3",
            "projected_gravity_3",
            "joint_position_relative_to_default_12",
            "joint_velocity_12",
            "previous_bounded_normalized_action_12",
            "gait_phase_sin_cos_2"};
        const auto exported_layout = load_string_vector(
            policy_config_["observation_layout"],
            expected_layout.size(),
            "observation_layout");
        if (exported_layout != expected_layout) {
            throw std::runtime_error(
                "47-D observation_layout does not match the supported append-only phase contract");
        }

        if (!policy_config_["gait_phase_enabled"] ||
            !policy_config_["gait_phase_enabled"].as<bool>()) {
            throw std::runtime_error("47-D policy requires gait_phase_enabled: true");
        }
        gait_phase_enabled_ = true;
        gait_phase_period_s_ = policy_config_["gait_phase_period_s"]
            ? policy_config_["gait_phase_period_s"].as<double>()
            : 0.0;
        gait_phase_encoding_ = policy_config_["gait_phase_encoding"]
            ? policy_config_["gait_phase_encoding"].as<std::string>()
            : std::string{};
        gait_phase_append_order_ = policy_config_["gait_phase_append_order"]
            ? policy_config_["gait_phase_append_order"].as<std::string>()
            : std::string{};
        gait_phase_training_timebase_ = policy_config_["gait_phase_training_timebase"]
            ? policy_config_["gait_phase_training_timebase"].as<std::string>()
            : std::string{};
        gait_phase_training_reset_semantics_ =
            policy_config_["gait_phase_training_reset_semantics"]
            ? policy_config_["gait_phase_training_reset_semantics"].as<std::string>()
            : std::string{};

        if (std::fabs(policy_dt_ - 0.02) > 1.0e-9) {
            throw std::runtime_error(
                "phase-guided observation contract v1 requires policy_dt: 0.02");
        }
        if (std::fabs(gait_phase_period_s_ - 0.72) > 1.0e-9) {
            throw std::runtime_error(
                "phase-guided observation contract v1 requires gait_phase_period_s: 0.72");
        }
        if (gait_phase_encoding_ != "sin_cos_2pi") {
            throw std::runtime_error(
                "phase-guided observation contract v1 requires gait_phase_encoding: sin_cos_2pi");
        }
        if (gait_phase_append_order_ != "after_previous_action") {
            throw std::runtime_error(
                "phase-guided observation contract v1 requires "
                "gait_phase_append_order: after_previous_action");
        }
        if (gait_phase_training_timebase_ != "episode_step_time") {
            throw std::runtime_error(
                "phase-guided observation contract v1 requires "
                "gait_phase_training_timebase: episode_step_time");
        }
        if (gait_phase_training_reset_semantics_ != "environment_episode_reset") {
            throw std::runtime_error(
                "phase-guided observation contract v1 requires "
                "gait_phase_training_reset_semantics: environment_episode_reset");
        }

        gait_phase_clock_.configure(gait_phase_period_s_, policy_dt_);
        RCLCPP_INFO(
            this->get_logger(),
            "Observation contract v2 validated: 47-D phase-guided layout, %.3fs period, %zu policy ticks.",
            gait_phase_period_s_, gait_phase_clock_.period_ticks());
    }

    void load_policy_config(const std::string& config_path)
    {
        RCLCPP_INFO(this->get_logger(), "Loading policy config from: %s", config_path.c_str());
        policy_config_ = YAML::LoadFile(config_path);

        num_observations_ = policy_config_["num_observations"]
            ? policy_config_["num_observations"].as<size_t>() : 45;
        num_actions_ = policy_config_["num_actions"]
            ? policy_config_["num_actions"].as<size_t>() : 12;
        policy_dt_ = policy_config_["policy_dt"]
            ? policy_config_["policy_dt"].as<double>() : 0.04;

        if (!is_finite(policy_dt_) || !(policy_dt_ > 0.0)) {
            throw std::runtime_error("policy_dt must be finite and positive");
        }

        load_observation_contract();

        action_limit_lower_ = load_float_vector(
            policy_config_["action_limit_lower"], num_actions_, "action_limit_lower");
        action_limit_upper_ = load_float_vector(
            policy_config_["action_limit_upper"], num_actions_, "action_limit_upper");

        if (policy_config_["action_contract_version"]) {
            action_contract_version_ = policy_config_["action_contract_version"].as<int>();
            if (action_contract_version_ != 3 && action_contract_version_ != 4) {
                throw std::runtime_error(
                    "Unsupported action_contract_version: " +
                    std::to_string(action_contract_version_) + ". Expected 3 or 4.");
            }

            const std::string contract_label =
                "Action contract v" + std::to_string(action_contract_version_);
            if (!policy_config_["action_transform"]) {
                throw std::runtime_error(
                    contract_label + " is missing required YAML field: action_transform");
            }
            action_transform_ = policy_config_["action_transform"].as<std::string>();
            const std::string expected_transform = action_contract_version_ == 4
                ? "bounded_default_centered_vector_residual"
                : "bounded_default_centered_symmetric_residual";
            if (action_transform_ != expected_transform) {
                throw std::runtime_error(
                    "Unsupported " + contract_label + " transform: " + action_transform_ +
                    ". Expected " + expected_transform + ".");
            }

            action_contract_name_ = policy_config_["action_contract_name"]
                ? policy_config_["action_contract_name"].as<std::string>() : std::string{};
            deployment_contract_profile_ = policy_config_["deployment_contract_profile"]
                ? policy_config_["deployment_contract_profile"].as<std::string>() : std::string{};

            action_residual_scale_rad_ = load_float_vector(
                policy_config_["action_residual_scale_rad"],
                num_actions_,
                "action_residual_scale_rad");
            exported_action_default_rad_ = load_float_vector(
                policy_config_["action_default_rad"],
                num_actions_,
                "action_default_rad");
            exported_action_target_lower_rad_ = load_float_vector(
                policy_config_["action_target_lower_rad"],
                num_actions_,
                "action_target_lower_rad");
            exported_action_target_upper_rad_ = load_float_vector(
                policy_config_["action_target_upper_rad"],
                num_actions_,
                "action_target_upper_rad");
            exported_action_indices_ = load_int_vector(
                policy_config_["action_indices"], num_actions_, "action_indices");

            if (policy_config_["action_nominal_residual_lower_rad"] ||
                policy_config_["action_nominal_residual_upper_rad"]) {
                exported_action_nominal_lower_rad_ = load_float_vector(
                    policy_config_["action_nominal_residual_lower_rad"],
                    num_actions_,
                    "action_nominal_residual_lower_rad");
                exported_action_nominal_upper_rad_ = load_float_vector(
                    policy_config_["action_nominal_residual_upper_rad"],
                    num_actions_,
                    "action_nominal_residual_upper_rad");
            } else if (action_contract_version_ == 4) {
                throw std::runtime_error(
                    "Action contract v4 requires action_nominal_residual_lower_rad and "
                    "action_nominal_residual_upper_rad");
            }

            const size_t num_joints = policy_config_["num_joints"]
                ? policy_config_["num_joints"].as<size_t>() : 0U;
            if (num_joints == 0U) {
                throw std::runtime_error(contract_label + " requires a positive num_joints field");
            }
            exported_sim_joint_names_ = load_string_vector(
                policy_config_["joints"], num_joints, "joints");
            exported_sim_default_joint_positions_ = load_float_vector(
                policy_config_["default_joint_positions"],
                num_joints,
                "default_joint_positions");

            if (!policy_config_["previous_action_observation"]) {
                throw std::runtime_error(
                    contract_label +
                    " is missing required YAML field: previous_action_observation");
            }
            previous_action_observation_ =
                policy_config_["previous_action_observation"].as<std::string>();
            if (previous_action_observation_ != "bounded_normalized_action") {
                throw std::runtime_error(
                    "Unsupported previous_action_observation for " + contract_label + ": " +
                    previous_action_observation_);
            }

            if (!policy_config_["deployment_requires_action_contract_transform"] ||
                !policy_config_["deployment_requires_action_contract_transform"].as<bool>()) {
                throw std::runtime_error(
                    contract_label +
                    " must declare deployment_requires_action_contract_transform: true");
            }
            if (action_contract_version_ == 4) {
                if (!policy_config_["deployment_requires_action_contract_v4_transform"] ||
                    !policy_config_["deployment_requires_action_contract_v4_transform"].as<bool>()) {
                    throw std::runtime_error(
                        "Action contract v4 must declare "
                        "deployment_requires_action_contract_v4_transform: true");
                }
                if (action_contract_name_.empty()) {
                    throw std::runtime_error(
                        "Action contract v4 requires a non-empty action_contract_name");
                }
                if (deployment_contract_profile_.empty()) {
                    throw std::runtime_error(
                        "Action contract v4 requires a non-empty deployment_contract_profile");
                }
            }
        } else {
            action_contract_version_ = 0;
            action_transform_ = "legacy_default_centered_scale";
            action_residual_scale_rad_ = load_float_vector(
                policy_config_["action_scale"], num_actions_, "action_scale");
            RCLCPP_WARN(
                this->get_logger(),
                "Legacy policy action schema detected (no action_contract_version). "
                "Use a paired action-contract-v3 or v4 deployment bundle for current hardware policy work.");
        }

        for (size_t i = 0; i < num_actions_; ++i) {
            if (action_limit_lower_[i] > action_limit_upper_[i]) {
                throw std::runtime_error("action limit lower > upper at index " + std::to_string(i));
            }
            if (!(action_residual_scale_rad_[i] > 0.0f)) {
                throw std::runtime_error(
                    "action residual scale must be positive at index " + std::to_string(i));
            }
        }

        const std::string contract_text = action_contract_version_ > 0
            ? ("v" + std::to_string(action_contract_version_))
            : "legacy";
        RCLCPP_INFO(
            this->get_logger(),
            "Policy config loaded: num_observations=%zu, observation_contract=v%d/%s, num_actions=%zu, policy_dt=%.3f, action_contract=%s, profile=%s",
            num_observations_,
            observation_contract_version_,
            observation_contract_name_.c_str(),
            num_actions_,
            policy_dt_,
            contract_text.c_str(),
            deployment_contract_profile_.empty() ? "none" : deployment_contract_profile_.c_str());
    }

    void load_joint_map(const std::string& joint_map_path)
    {
        RCLCPP_INFO(this->get_logger(), "Loading joint map from: %s", joint_map_path.c_str());
        const YAML::Node joint_map = YAML::LoadFile(joint_map_path);
        const auto joints_node = joint_map["joints"];
        if (!joints_node || !joints_node.IsSequence()) {
            throw std::runtime_error("joint_map.yaml is missing required sequence: joints");
        }

        std::vector<JointMapEntry> entries;
        entries.reserve(joints_node.size());

        for (const auto& joint : joints_node) {
            JointMapEntry entry;
            entry.name = joint["name"].as<std::string>();
            entry.policy_action_index = joint["policy_action_index"].as<int>();
            entry.sim_joint_index = joint["sim_joint_index"].as<int>();
            entry.ros_joint_state_index = joint["ros_joint_state_index"]
                ? joint["ros_joint_state_index"].as<int>() : entry.policy_action_index;
            entry.micro_ros_array_index = joint["micro_ros_array_index"]
                ? joint["micro_ros_array_index"].as<int>() : entry.policy_action_index;
            entry.servo_id = joint["servo_id"] ? joint["servo_id"].as<int>() : -1;
            entry.default_joint_rad = joint["default_joint_rad"].as<float>();
            entry.limit_lower_rad = joint["limit_lower_rad"].as<float>();
            entry.limit_upper_rad = joint["limit_upper_rad"].as<float>();
            entries.push_back(entry);
        }

        std::sort(entries.begin(), entries.end(), [](const JointMapEntry& lhs, const JointMapEntry& rhs) {
            return lhs.policy_action_index < rhs.policy_action_index;
        });

        if (entries.size() != num_actions_) {
            throw std::runtime_error(
                "joint_map.yaml joint count mismatch: expected " + std::to_string(num_actions_) +
                ", got " + std::to_string(entries.size()));
        }

        joint_names_.clear();
        sim_joint_indices_.clear();
        ros_joint_state_indices_.clear();
        micro_ros_array_indices_.clear();
        servo_ids_.clear();
        default_joint_positions_.clear();
        joint_lower_limits_.clear();
        joint_upper_limits_.clear();

        for (size_t i = 0; i < entries.size(); ++i) {
            const auto& entry = entries[i];
            if (entry.policy_action_index != static_cast<int>(i)) {
                throw std::runtime_error(
                    "joint_map.yaml policy_action_index values must be contiguous 0..num_actions-1");
            }
            if (!is_finite(entry.default_joint_rad) ||
                !is_finite(entry.limit_lower_rad) ||
                !is_finite(entry.limit_upper_rad) ||
                entry.limit_lower_rad > entry.limit_upper_rad) {
                throw std::runtime_error("invalid joint map values for " + entry.name);
            }

            joint_names_.push_back(entry.name);
            sim_joint_indices_.push_back(entry.sim_joint_index);
            ros_joint_state_indices_.push_back(entry.ros_joint_state_index);
            micro_ros_array_indices_.push_back(entry.micro_ros_array_index);
            servo_ids_.push_back(entry.servo_id);
            default_joint_positions_.push_back(entry.default_joint_rad);
            joint_lower_limits_.push_back(entry.limit_lower_rad);
            joint_upper_limits_.push_back(entry.limit_upper_rad);
        }

        RCLCPP_INFO(this->get_logger(), "Joint map loaded in policy action order:");
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            RCLCPP_INFO(
                this->get_logger(),
                "  action[%zu] -> sim_joint[%d] %s -> servo_id %d, default %.3f rad, limits [%.3f, %.3f]",
                i,
                sim_joint_indices_[i],
                joint_names_[i].c_str(),
                servo_ids_[i],
                default_joint_positions_[i],
                joint_lower_limits_[i],
                joint_upper_limits_[i]);
        }
    }

    void validate_policy_contract() const
    {
        if (!littlegreen_biped::is_supported_observation_count(num_observations_) ||
            num_actions_ != littlegreen_biped::kNumPolicyActions) {
            throw std::runtime_error(
                "Expected a supported policy interface obs[45|47] -> actions[12], got obs[" +
                std::to_string(num_observations_) + "] -> actions[" +
                std::to_string(num_actions_) + "]");
        }
        if (num_observations_ == littlegreen_biped::kPhaseGuidedObservationCount &&
            action_contract_version_ != 4) {
            throw std::runtime_error(
                "47-D phase-guided policies require action_contract_version: 4");
        }

        const size_t n = num_actions_;
        if (joint_names_.size() != n ||
            sim_joint_indices_.size() != n ||
            micro_ros_array_indices_.size() != n ||
            default_joint_positions_.size() != n ||
            joint_lower_limits_.size() != n ||
            joint_upper_limits_.size() != n ||
            action_residual_scale_rad_.size() != n ||
            action_limit_lower_.size() != n ||
            action_limit_upper_.size() != n) {
            throw std::runtime_error("joint map / policy vector size mismatch");
        }

        if (action_contract_version_ != 3 && action_contract_version_ != 4) {
            return;
        }

        const std::string contract_label =
            "action contract v" + std::to_string(action_contract_version_);
        if (exported_action_default_rad_.size() != n ||
            exported_action_target_lower_rad_.size() != n ||
            exported_action_target_upper_rad_.size() != n ||
            exported_action_indices_.size() != n) {
            throw std::runtime_error(contract_label + " vector size mismatch");
        }

        constexpr float tolerance_rad = 1.0e-5f;
        const auto require_close = [tolerance_rad](
            float exported,
            float hardware,
            const std::string& field,
            const std::string& joint_name,
            size_t index) {
            if (std::fabs(exported - hardware) > tolerance_rad) {
                std::ostringstream stream;
                stream << std::setprecision(10)
                       << "Policy/joint-map mismatch for " << field
                       << " at action[" << index << "] " << joint_name
                       << ": exported=" << exported
                       << ", joint_map=" << hardware
                       << ", tolerance=" << tolerance_rad;
                throw std::runtime_error(stream.str());
            }
        };

        const auto [minimum_scale, maximum_scale] = std::minmax_element(
            action_residual_scale_rad_.begin(), action_residual_scale_rad_.end());
        const bool scales_are_nonuniform =
            (*maximum_scale - *minimum_scale) > tolerance_rad;
        if (action_contract_version_ == 3 && scales_are_nonuniform) {
            throw std::runtime_error(
                "Action contract v3 requires a uniform symmetric residual scale; "
                "use action contract v4 for a per-joint vector residual profile");
        }
        if (action_contract_version_ == 4 && !scales_are_nonuniform) {
            throw std::runtime_error(
                "Action contract v4 requires a non-uniform per-joint residual scale vector");
        }

        if (action_contract_version_ == 4 &&
            (exported_action_nominal_lower_rad_.size() != n ||
             exported_action_nominal_upper_rad_.size() != n)) {
            throw std::runtime_error("action contract v4 nominal residual vector size mismatch");
        }

        for (size_t i = 0; i < n; ++i) {
            if (exported_action_indices_[i] != sim_joint_indices_[i]) {
                throw std::runtime_error(
                    "Policy/joint-map action_indices mismatch at action[" +
                    std::to_string(i) + "] " + joint_names_[i] +
                    ": exported sim index " + std::to_string(exported_action_indices_[i]) +
                    ", joint_map sim index " + std::to_string(sim_joint_indices_[i]));
            }

            const int sim_index = exported_action_indices_[i];
            if (sim_index < 0 ||
                static_cast<size_t>(sim_index) >= exported_sim_joint_names_.size() ||
                static_cast<size_t>(sim_index) >= exported_sim_default_joint_positions_.size()) {
                throw std::runtime_error(
                    "Policy action_indices contains out-of-range sim index at action[" +
                    std::to_string(i) + "]");
            }

            if (exported_sim_joint_names_[static_cast<size_t>(sim_index)] != joint_names_[i]) {
                throw std::runtime_error(
                    "Policy/joint-map joint-name mismatch at action[" +
                    std::to_string(i) + "]: exported=" +
                    exported_sim_joint_names_[static_cast<size_t>(sim_index)] +
                    ", joint_map=" + joint_names_[i]);
            }

            require_close(
                exported_action_default_rad_[i],
                default_joint_positions_[i],
                "action_default_rad",
                joint_names_[i],
                i);
            require_close(
                exported_sim_default_joint_positions_[static_cast<size_t>(sim_index)],
                default_joint_positions_[i],
                "default_joint_positions[action_indices]",
                joint_names_[i],
                i);
            require_close(
                exported_action_target_lower_rad_[i],
                joint_lower_limits_[i],
                "action_target_lower_rad",
                joint_names_[i],
                i);
            require_close(
                exported_action_target_upper_rad_[i],
                joint_upper_limits_[i],
                "action_target_upper_rad",
                joint_names_[i],
                i);

            if (std::fabs(action_limit_lower_[i] + 1.0f) > tolerance_rad ||
                std::fabs(action_limit_upper_[i] - 1.0f) > tolerance_rad) {
                throw std::runtime_error(
                    "Action contracts v3/v4 require normalized action limits [-1, 1] at action[" +
                    std::to_string(i) + "] " + joint_names_[i]);
            }
            if (exported_action_target_lower_rad_[i] > exported_action_default_rad_[i] ||
                exported_action_default_rad_[i] > exported_action_target_upper_rad_[i]) {
                throw std::runtime_error(
                    contract_label + " default is outside physical bounds at action[" +
                    std::to_string(i) + "] " + joint_names_[i]);
            }

            if (action_contract_version_ == 4) {
                const float expected_nominal_lower = std::max(
                    exported_action_target_lower_rad_[i],
                    exported_action_default_rad_[i] - action_residual_scale_rad_[i]);
                const float expected_nominal_upper = std::min(
                    exported_action_target_upper_rad_[i],
                    exported_action_default_rad_[i] + action_residual_scale_rad_[i]);
                require_close(
                    exported_action_nominal_lower_rad_[i],
                    expected_nominal_lower,
                    "action_nominal_residual_lower_rad",
                    joint_names_[i],
                    i);
                require_close(
                    exported_action_nominal_upper_rad_[i],
                    expected_nominal_upper,
                    "action_nominal_residual_upper_rad",
                    joint_names_[i],
                    i);
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Action contract v%d validated against joint_map.yaml: defaults, physical bounds, residual scales, joint names, action indices%s match.",
            action_contract_version_,
            action_contract_version_ == 4 ? ", and nominal residual bounds" : "");
    }

    void initialize_state_storage()
    {
        prev_actions_.assign(num_actions_, 0.0f);
        joint_positions_.assign(num_actions_, 0.0f);
        joint_velocities_.assign(num_actions_, 0.0f);
        base_ang_vel_.assign(3, 0.0f);
        cmd_vel_.assign(3, 0.0f);

        joint_position_seen_.assign(num_actions_, false);
        joint_velocity_seen_.assign(num_actions_, false);
        joint_position_update_time_.assign(num_actions_, SteadyClock::time_point{});
        joint_velocity_update_time_.assign(num_actions_, SteadyClock::time_point{});

        joint_feedback_age_ms_.assign(num_actions_, UINT32_MAX);
        joint_feedback_age_seen_.assign(num_actions_, false);
    }

    std::string resolve_model_path(
        const std::string& policy_config_path,
        const std::string& onnx_model_path_override) const
    {
        if (!onnx_model_path_override.empty()) {
            if (!file_exists(onnx_model_path_override)) {
                throw std::runtime_error(
                    "onnx_model_path override does not exist: " + onnx_model_path_override);
            }
            return onnx_model_path_override;
        }

        const std::string config_dir = parent_directory(policy_config_path);

        // Deployment path: the ONNX model is paired beside policy_latest.yaml.
        if (policy_config_["policy_checkpoint_relative_path"]) {
            const std::string relative_path =
                policy_config_["policy_checkpoint_relative_path"].as<std::string>();
            const std::string candidate = config_dir + "/" + relative_path;
            if (!file_exists(candidate)) {
                throw std::runtime_error(
                    "paired ONNX model is missing beside policy YAML: " + candidate);
            }
            return candidate;
        }

        if (policy_config_["policy_checkpoint_filename"]) {
            const std::string filename =
                policy_config_["policy_checkpoint_filename"].as<std::string>();
            const std::string candidate = config_dir + "/" + filename;
            if (file_exists(candidate)) {
                return candidate;
            }
        }

        // Legacy compatibility only. We do not silently fall back to an unrelated packaged model.
        if (policy_config_["policy_checkpoint_path"]) {
            const std::string absolute_path =
                policy_config_["policy_checkpoint_path"].as<std::string>();
            if (file_exists(absolute_path)) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Using legacy absolute policy path. Prefer a paired relative ONNX artifact: %s",
                    absolute_path.c_str());
                return absolute_path;
            }
        }

        throw std::runtime_error(
            "No paired ONNX model could be resolved from policy config: " + policy_config_path);
    }

    void load_onnx_model(const std::string& model_path)
    {
        RCLCPP_INFO(this->get_logger(), "Loading ONNX model from: %s", model_path.c_str());

        if (policy_config_["policy_sha256"]) {
            const std::string expected_sha256 = policy_config_["policy_sha256"].as<std::string>();
            const std::string actual_sha256 = sha256_file(model_path);
            if (actual_sha256 != expected_sha256) {
                throw std::runtime_error(
                    "ONNX SHA-256 mismatch. YAML expects " + expected_sha256 +
                    ", file is " + actual_sha256);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Policy artifact checksum verified: %s",
                actual_sha256.c_str());
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "policy_sha256 missing from YAML; artifact pairing cannot be checksum-verified.");
        }

        try {
            session_options_.SetIntraOpNumThreads(1);
            session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
            allocator_ = std::make_unique<Ort::AllocatorWithDefaultOptions>();

            if (session_->GetInputCount() != 1U || session_->GetOutputCount() != 1U) {
                throw std::runtime_error(
                    "ONNX policy must expose exactly one input and one output tensor");
            }

            Ort::AllocatorWithDefaultOptions ort_allocator;
            input_name_ = std::string(session_->GetInputNameAllocated(0, ort_allocator).get());
            output_name_ = std::string(session_->GetOutputNameAllocated(0, ort_allocator).get());

            const auto input_info = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
            const auto output_info = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
            const auto input_shape = input_info.GetShape();
            const auto output_shape = output_info.GetShape();

            if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                throw std::runtime_error("ONNX policy input and output tensors must be float32");
            }
            if (input_shape.size() != 2U ||
                input_shape.back() != static_cast<int64_t>(num_observations_) ||
                (input_shape.front() > 0 && input_shape.front() != 1)) {
                throw std::runtime_error(
                    "ONNX input tensor must have shape [1," +
                    std::to_string(num_observations_) + "] (dynamic batch is allowed)");
            }
            if (output_shape.size() != 2U ||
                output_shape.back() != static_cast<int64_t>(num_actions_) ||
                (output_shape.front() > 0 && output_shape.front() != 1)) {
                throw std::runtime_error(
                    "ONNX output tensor must have shape [1," +
                    std::to_string(num_actions_) + "] (dynamic batch is allowed)");
            }

            RCLCPP_INFO(
                this->get_logger(),
                "ONNX model loaded. Input='%s' shape=[%lld,%lld], Output='%s' shape=[%lld,%lld].",
                input_name_.c_str(),
                static_cast<long long>(input_shape[0]),
                static_cast<long long>(input_shape[1]),
                output_name_.c_str(),
                static_cast<long long>(output_shape[0]),
                static_cast<long long>(output_shape[1]));
        } catch (const Ort::Exception& error) {
            RCLCPP_FATAL(this->get_logger(), "ONNX Runtime failed: %s", error.what());
            throw;
        }
    }

    Eigen::Vector3f imu_to_base_vector(const Eigen::Vector3f& imu_vector) const
    {
        Eigen::Matrix3f rotation;
        rotation <<
            imu_to_base_matrix_[0], imu_to_base_matrix_[1], imu_to_base_matrix_[2],
            imu_to_base_matrix_[3], imu_to_base_matrix_[4], imu_to_base_matrix_[5],
            imu_to_base_matrix_[6], imu_to_base_matrix_[7], imu_to_base_matrix_[8];
        return rotation * imu_vector;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        const std::array<float, 3> angular_velocity_imu{
            static_cast<float>(msg->angular_velocity.x),
            static_cast<float>(msg->angular_velocity.y),
            static_cast<float>(msg->angular_velocity.z)};
        const std::array<float, 4> orientation_wxyz{
            static_cast<float>(msg->orientation.w),
            static_cast<float>(msg->orientation.x),
            static_cast<float>(msg->orientation.y),
            static_cast<float>(msg->orientation.z)};

        if (!all_finite(angular_velocity_imu) || !all_finite(orientation_wxyz)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejected IMU sample containing non-finite values.");
            return;
        }

        Eigen::Quaternionf q(
            orientation_wxyz[0], orientation_wxyz[1], orientation_wxyz[2], orientation_wxyz[3]);
        const float q_norm = q.norm();
        if (!is_finite(q_norm) || q_norm < 1.0e-6f) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejected IMU sample with invalid quaternion norm.");
            return;
        }
        q.normalize();

        const Eigen::Vector3f omega_imu(
            angular_velocity_imu[0], angular_velocity_imu[1], angular_velocity_imu[2]);
        const Eigen::Vector3f omega_base = imu_to_base_vector(omega_imu);
        if (!omega_base.allFinite()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejected IMU sample after IMU->base transform produced non-finite angular velocity.");
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        base_ang_vel_[0] = omega_base.x();
        base_ang_vel_[1] = omega_base.y();
        base_ang_vel_[2] = omega_base.z();
        imu_orientation_wxyz_ = {q.w(), q.x(), q.y(), q.z()};
        have_valid_imu_ = true;
        last_imu_update_time_ = SteadyClock::now();
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        const auto receipt_time = SteadyClock::now();
        size_t position_updates = 0;
        size_t velocity_updates = 0;
        size_t rejected_nonfinite = 0;

        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!msg->name.empty()) {
            std::unordered_map<std::string, size_t> source_index_by_name;
            source_index_by_name.reserve(msg->name.size());
            for (size_t i = 0; i < msg->name.size(); ++i) {
                source_index_by_name[msg->name[i]] = i;
            }

            for (size_t i = 0; i < joint_names_.size(); ++i) {
                const auto it = source_index_by_name.find(joint_names_[i]);
                if (it == source_index_by_name.end()) {
                    continue;
                }

                const size_t source_index = it->second;
                if (source_index < msg->position.size()) {
                    const float value = static_cast<float>(msg->position[source_index]);
                    if (is_finite(value)) {
                        joint_positions_[i] = value;
                        joint_position_seen_[i] = true;
                        joint_position_update_time_[i] = receipt_time;
                        ++position_updates;
                    } else {
                        ++rejected_nonfinite;
                    }
                }

                if (source_index < msg->velocity.size()) {
                    const float value = static_cast<float>(msg->velocity[source_index]);
                    if (is_finite(value)) {
                        joint_velocities_[i] = value;
                        joint_velocity_seen_[i] = true;
                        joint_velocity_update_time_[i] = receipt_time;
                        ++velocity_updates;
                    } else {
                        ++rejected_nonfinite;
                    }
                }
            }
        } else {
            update_unnamed_joint_state(*msg, receipt_time, position_updates, velocity_updates, rejected_nonfinite);
        }

        if (position_updates == 0 && velocity_updates == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "JointState sample did not update any policy joints (names=%zu, position=%zu, velocity=%zu).",
                msg->name.size(), msg->position.size(), msg->velocity.size());
        }

        if (rejected_nonfinite > 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejected %zu non-finite JointState field(s); freshness for those joints was not updated.",
                rejected_nonfinite);
        }
    }

    void update_unnamed_joint_state(
        const sensor_msgs::msg::JointState& msg,
        const SteadyClock::time_point receipt_time,
        size_t& position_updates,
        size_t& velocity_updates,
        size_t& rejected_nonfinite)
    {
        auto update_position = [&](size_t policy_index, size_t source_index) {
            if (source_index >= msg.position.size()) {
                return;
            }
            const float value = static_cast<float>(msg.position[source_index]);
            if (is_finite(value)) {
                joint_positions_[policy_index] = value;
                joint_position_seen_[policy_index] = true;
                joint_position_update_time_[policy_index] = receipt_time;
                ++position_updates;
            } else {
                ++rejected_nonfinite;
            }
        };

        auto update_velocity = [&](size_t policy_index, size_t source_index) {
            if (source_index >= msg.velocity.size()) {
                return;
            }
            const float value = static_cast<float>(msg.velocity[source_index]);
            if (is_finite(value)) {
                joint_velocities_[policy_index] = value;
                joint_velocity_seen_[policy_index] = true;
                joint_velocity_update_time_[policy_index] = receipt_time;
                ++velocity_updates;
            } else {
                ++rejected_nonfinite;
            }
        };

        if (msg.position.size() == num_actions_) {
            for (size_t i = 0; i < num_actions_; ++i) {
                update_position(i, i);
            }
        } else if (msg.position.size() >= 14) {
            for (size_t i = 0; i < num_actions_; ++i) {
                const int sim_index = sim_joint_indices_[i];
                if (sim_index >= 0) {
                    update_position(i, static_cast<size_t>(sim_index));
                }
            }
        }

        if (msg.velocity.size() == num_actions_) {
            for (size_t i = 0; i < num_actions_; ++i) {
                update_velocity(i, i);
            }
        } else if (msg.velocity.size() >= 14) {
            for (size_t i = 0; i < num_actions_; ++i) {
                const int sim_index = sim_joint_indices_[i];
                if (sim_index >= 0) {
                    update_velocity(i, static_cast<size_t>(sim_index));
                }
            }
        }
    }

    void joint_feedback_age_callback(
        const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        std::vector<uint32_t> mapped_age_ms(num_actions_, UINT32_MAX);
        for (size_t i = 0; i < num_actions_; ++i) {
            const int source_index = micro_ros_array_indices_[i];
            if (source_index < 0 ||
                static_cast<size_t>(source_index) >= msg->data.size()) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejected /joint_feedback_age_ms: expected mapped index %d for %s, array size=%zu.",
                    source_index, joint_names_[i].c_str(), msg->data.size());
                return;
            }
            mapped_age_ms[i] = msg->data[static_cast<size_t>(source_index)];
        }

        const auto receipt_time = SteadyClock::now();
        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_feedback_age_ms_ = mapped_age_ms;
        for (size_t i = 0; i < num_actions_; ++i) {
            joint_feedback_age_seen_[i] = mapped_age_ms[i] != UINT32_MAX;
        }
        have_joint_feedback_age_message_ = true;
        last_joint_feedback_age_update_time_ = receipt_time;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!msg) {
            return;
        }

        const std::array<float, 3> command{
            static_cast<float>(msg->linear.x),
            static_cast<float>(msg->linear.y),
            static_cast<float>(msg->angular.z)};

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!all_finite(command)) {
            cmd_vel_ = {0.0f, 0.0f, 0.0f};
            have_valid_command_ = false;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejected non-finite /command_velocity and forced the command observation to zero.");
            return;
        }

        cmd_vel_[0] = command[0];
        cmd_vel_[1] = command[1];
        cmd_vel_[2] = command[2];
        have_valid_command_ = true;
        last_command_update_time_ = SteadyClock::now();
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr /*msg*/)
    {
        // /command_velocity is the canonical command input.
    }

    bool make_ready_snapshot(PolicyStateSnapshot& snapshot, std::string& not_ready_reason)
    {
        const auto now = SteadyClock::now();
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!override_imu_) {
            if (!have_valid_imu_) {
                not_ready_reason = "waiting for first valid IMU sample";
                return false;
            }
            const double imu_age = std::chrono::duration<double>(now - last_imu_update_time_).count();
            if (imu_age > imu_timeout_sec_) {
                std::ostringstream stream;
                stream << "IMU stale: age=" << imu_age << "s limit=" << imu_timeout_sec_ << "s";
                not_ready_reason = stream.str();
                return false;
            }
        }

        double max_joint_age = 0.0;
        for (size_t i = 0; i < num_actions_; ++i) {
            if (!joint_position_seen_[i]) {
                not_ready_reason = "missing joint position: " + joint_names_[i];
                return false;
            }
            const double position_age =
                std::chrono::duration<double>(now - joint_position_update_time_[i]).count();
            max_joint_age = std::max(max_joint_age, position_age);
            if (position_age > joint_state_timeout_sec_) {
                std::ostringstream stream;
                stream << "joint position stale: " << joint_names_[i]
                       << " age=" << position_age << "s limit=" << joint_state_timeout_sec_ << "s";
                not_ready_reason = stream.str();
                return false;
            }

            if (require_joint_velocity_) {
                if (!joint_velocity_seen_[i]) {
                    not_ready_reason = "missing joint velocity: " + joint_names_[i];
                    return false;
                }
                const double velocity_age =
                    std::chrono::duration<double>(now - joint_velocity_update_time_[i]).count();
                max_joint_age = std::max(max_joint_age, velocity_age);
                if (velocity_age > joint_state_timeout_sec_) {
                    std::ostringstream stream;
                    stream << "joint velocity stale: " << joint_names_[i]
                           << " age=" << velocity_age << "s limit=" << joint_state_timeout_sec_ << "s";
                    not_ready_reason = stream.str();
                    return false;
                }
            }
        }

        if (!use_sim_ && require_joint_feedback_age_) {
            if (!have_joint_feedback_age_message_) {
                not_ready_reason = "waiting for /joint_feedback_age_ms";
                return false;
            }

            const double age_topic_receipt_age =
                std::chrono::duration<double>(
                    now - last_joint_feedback_age_update_time_).count();
            if (age_topic_receipt_age > joint_feedback_age_topic_timeout_sec_) {
                std::ostringstream stream;
                stream << "joint feedback-age topic stale: age=" << age_topic_receipt_age
                       << "s limit=" << joint_feedback_age_topic_timeout_sec_ << "s";
                not_ready_reason = stream.str();
                return false;
            }

            for (size_t i = 0; i < num_actions_; ++i) {
                if (!joint_feedback_age_seen_[i] ||
                    joint_feedback_age_ms_[i] == UINT32_MAX) {
                    not_ready_reason =
                        "no successful hardware feedback yet: " + joint_names_[i];
                    return false;
                }

                // The MCU-provided value is age at publication time. Adding the
                // host time since receipt makes the gate conservative between
                // age-topic updates without coupling it to the MCU read period.
                const double hardware_age_sec =
                    static_cast<double>(joint_feedback_age_ms_[i]) * 0.001 +
                    age_topic_receipt_age;
                if (hardware_age_sec > joint_feedback_max_age_sec_) {
                    std::ostringstream stream;
                    stream << "hardware joint feedback stale: " << joint_names_[i]
                           << " age=" << hardware_age_sec
                           << "s limit=" << joint_feedback_max_age_sec_ << "s";
                    not_ready_reason = stream.str();
                    return false;
                }
            }
        }

        snapshot.cmd_vel = cmd_vel_;
        if (zero_command_on_timeout_) {
            const bool command_missing = !have_valid_command_;
            const bool command_stale = have_valid_command_ &&
                std::chrono::duration<double>(now - last_command_update_time_).count() > command_timeout_sec_;
            if (command_missing || command_stale) {
                snapshot.cmd_vel = {0.0f, 0.0f, 0.0f};
            }
        }

        snapshot.base_ang_vel = base_ang_vel_;
        snapshot.imu_orientation_wxyz = imu_orientation_wxyz_;
        snapshot.joint_positions = joint_positions_;
        snapshot.joint_velocities = joint_velocities_;
        snapshot.prev_actions = prev_actions_;

        (void)max_joint_age;
        return true;
    }

    std::array<float, 3> compute_projected_gravity_base(
        const std::array<float, 4>& orientation_wxyz) const
    {
        Eigen::Quaternionf q_world_imu(
            orientation_wxyz[0], orientation_wxyz[1], orientation_wxyz[2], orientation_wxyz[3]);
        q_world_imu.normalize();

        const Eigen::Vector3f gravity_world(0.0f, 0.0f, -1.0f);
        const Eigen::Vector3f gravity_imu = q_world_imu.conjugate() * gravity_world;
        const Eigen::Vector3f gravity_base = imu_to_base_vector(gravity_imu);
        return {gravity_base.x(), gravity_base.y(), gravity_base.z()};
    }

    std::vector<float> compute_relative_joint_positions(
        const std::vector<float>& joint_positions) const
    {
        std::vector<float> relative;
        relative.reserve(num_actions_);
        for (size_t i = 0; i < num_actions_; ++i) {
            relative.push_back(joint_positions[i] - default_joint_positions_[i]);
        }
        return relative;
    }

    struct PolicyPostprocessResult
    {
        std::vector<float> clipped_raw_actions;
        std::vector<float> target_unclipped;
        std::vector<float> target_clipped;
        std::vector<uint8_t> saturation_mask;
    };

    PolicyPostprocessResult compute_target_joint_radians(
        const std::vector<float>& raw_actions) const
    {
        if (raw_actions.size() != num_actions_ || !all_finite(raw_actions)) {
            throw std::runtime_error("raw policy action vector is invalid");
        }

        PolicyPostprocessResult result;
        result.clipped_raw_actions.reserve(num_actions_);
        result.target_unclipped.reserve(num_actions_);
        result.target_clipped.reserve(num_actions_);
        result.saturation_mask.reserve(num_actions_);

        // saturation_mask bits:
        //   bit 0 (1): raw action clipped by action_limit_[lower|upper]
        //   bit 1 (2): scaled target clipped at physical lower joint limit
        //   bit 2 (4): scaled target clipped at physical upper joint limit
        for (size_t i = 0; i < num_actions_; ++i) {
            const float clipped_raw = clamp_float(
                raw_actions[i], action_limit_lower_[i], action_limit_upper_[i]);
            const float target_unclipped =
                default_joint_positions_[i] + action_residual_scale_rad_[i] * clipped_raw;
            const float target_clipped = clamp_float(
                target_unclipped, joint_lower_limits_[i], joint_upper_limits_[i]);

            if (!is_finite(target_clipped)) {
                throw std::runtime_error(
                    "non-finite target joint value at index " + std::to_string(i));
            }

            uint8_t mask = 0U;
            if (clipped_raw != raw_actions[i]) {
                mask |= 0x01U;
            }
            if (target_unclipped < joint_lower_limits_[i]) {
                mask |= 0x02U;
            }
            if (target_unclipped > joint_upper_limits_[i]) {
                mask |= 0x04U;
            }

            result.clipped_raw_actions.push_back(clipped_raw);
            result.target_unclipped.push_back(target_unclipped);
            result.target_clipped.push_back(target_clipped);
            result.saturation_mask.push_back(mask);
        }

        return result;
    }

    void publish_debug_float_array(
        const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& publisher,
        const std::vector<float>& values,
        const std::string& label) const
    {
        if (!publisher) {
            return;
        }
        std_msgs::msg::Float64MultiArray message;
        std_msgs::msg::MultiArrayDimension dimension;
        dimension.label = label;
        dimension.size = static_cast<uint32_t>(values.size());
        dimension.stride = static_cast<uint32_t>(values.size());
        message.layout.dim.push_back(dimension);
        message.data.reserve(values.size());
        for (const float value : values) {
            message.data.push_back(static_cast<double>(value));
        }
        publisher->publish(message);
    }

    void publish_policy_debug_observation(const std::vector<float>& observation) const
    {
        if (!publish_policy_debug_) {
            return;
        }
        publish_debug_float_array(
            policy_debug_observation_pub_,
            observation,
            "obs[" + std::to_string(observation.size()) + "]");
    }

    PhaseSnapshot capture_gait_phase() const
    {
        std::lock_guard<std::mutex> lock(gait_phase_mutex_);
        PhaseSnapshot snapshot;
        snapshot.sample = gait_phase_clock_.sample();
        snapshot.generation = gait_phase_generation_;
        return snapshot;
    }

    void publish_policy_debug_gait_phase(const GaitPhaseSample& phase) const
    {
        if (!publish_policy_debug_ || !policy_debug_gait_phase_pub_) {
            return;
        }
        std_msgs::msg::Float64MultiArray message;
        std_msgs::msg::MultiArrayDimension dimension;
        dimension.label =
            "phase,tick,period_ticks,sin,cos,expected_half_cycle(0=Lstance,1=Rstance)";
        dimension.size = 6U;
        dimension.stride = 6U;
        message.layout.dim.push_back(dimension);
        message.data = {
            static_cast<double>(phase.phase),
            static_cast<double>(phase.tick),
            static_cast<double>(phase.period_ticks),
            static_cast<double>(phase.sine),
            static_cast<double>(phase.cosine),
            static_cast<double>(phase.expected_half_cycle)};
        policy_debug_gait_phase_pub_->publish(message);
    }

    void advance_gait_phase_if_unchanged(const PhaseSnapshot& snapshot)
    {
        if (!gait_phase_enabled_) {
            return;
        }
        std::lock_guard<std::mutex> lock(gait_phase_mutex_);
        if (gait_phase_generation_ == snapshot.generation &&
            gait_phase_clock_.tick() == snapshot.sample.tick) {
            gait_phase_clock_.advance();
        }
    }

    void reset_gait_phase_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!response) {
            return;
        }
        if (!gait_phase_enabled_) {
            response->success = false;
            response->message = "active policy does not use the 47-D gait-phase contract";
            return;
        }
        if (policy_output_mode_ == "live") {
            response->success = false;
            response->message =
                "gait phase reset is refused in live mode; stop and restart the guarded live policy";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(gait_phase_mutex_);
            gait_phase_clock_.reset();
            ++gait_phase_generation_;
        }
        response->success = true;
        response->message = "gait phase reset to phase zero [sin,cos]=[0,1]";
        RCLCPP_WARN(
            this->get_logger(),
            "Gait phase explicitly reset in %s mode.",
            policy_output_mode_.c_str());
    }

    void publish_policy_debug_actions(
        const std::vector<float>& raw_actions,
        const PolicyPostprocessResult& result) const
    {
        if (!publish_policy_debug_) {
            return;
        }

        publish_debug_float_array(
            policy_debug_raw_action_pub_, raw_actions, "raw_action[12]");
        publish_debug_float_array(
            policy_debug_clipped_raw_action_pub_,
            result.clipped_raw_actions,
            "clipped_raw_action[12]");
        publish_debug_float_array(
            policy_debug_target_unclipped_pub_,
            result.target_unclipped,
            "target_unclipped_rad[12]");
        publish_debug_float_array(
            policy_debug_target_clipped_pub_,
            result.target_clipped,
            "target_clipped_rad[12]");

        if (policy_debug_saturation_mask_pub_) {
            std_msgs::msg::UInt8MultiArray mask_message;
            std_msgs::msg::MultiArrayDimension dimension;
            dimension.label = "bits:1=raw_clip,2=lower_limit,4=upper_limit";
            dimension.size = static_cast<uint32_t>(result.saturation_mask.size());
            dimension.stride = static_cast<uint32_t>(result.saturation_mask.size());
            mask_message.layout.dim.push_back(dimension);
            mask_message.data = result.saturation_mask;
            policy_debug_saturation_mask_pub_->publish(mask_message);
        }
    }

    void publish_policy_status(bool ready, const std::string& reason)
    {
        if (status_initialized_ && ready == last_ready_state_ && reason == last_status_reason_) {
            return;
        }

        std_msgs::msg::Bool ready_msg;
        ready_msg.data = ready;
        policy_ready_pub_->publish(ready_msg);

        std_msgs::msg::String status_msg;
        status_msg.data = ready ? ("READY: " + reason) : ("NOT_READY: " + reason);
        policy_status_pub_->publish(status_msg);

        if (ready && (!status_initialized_ || !last_ready_state_)) {
            RCLCPP_INFO(this->get_logger(), "Policy readiness gate opened.");
        } else if (!ready) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Policy inference gated: %s", reason.c_str());
        }

        status_initialized_ = true;
        last_ready_state_ = ready;
        last_status_reason_ = reason;
    }

    void run_policy_and_publish()
    {
        PolicyStateSnapshot snapshot;
        std::string not_ready_reason;
        if (!make_ready_snapshot(snapshot, not_ready_reason)) {
            publish_policy_status(false, not_ready_reason);
            return;
        }

        if (policy_output_mode_ == "disabled") {
            publish_policy_status(true, "policy output disabled");
            return;
        }

        PhaseSnapshot phase_snapshot;
        const GaitPhaseSample* gait_phase = nullptr;
        if (gait_phase_enabled_) {
            phase_snapshot = capture_gait_phase();
            gait_phase = &phase_snapshot.sample;
        }

        std::array<float, 3> projected_gravity{0.0f, 0.0f, -1.0f};
        std::vector<float> policy_base_angular_velocity = snapshot.base_ang_vel;
        if (override_imu_) {
            policy_base_angular_velocity = {0.0f, 0.0f, 0.0f};
        } else {
            projected_gravity = compute_projected_gravity_base(
                snapshot.imu_orientation_wxyz);
        }

        const auto relative_joint_positions =
            compute_relative_joint_positions(snapshot.joint_positions);

        std::vector<float> input_tensor_values;
        try {
            input_tensor_values = littlegreen_biped::build_policy_observation(
                snapshot.cmd_vel,
                policy_base_angular_velocity,
                projected_gravity,
                relative_joint_positions,
                snapshot.joint_velocities,
                snapshot.prev_actions,
                gait_phase);
        } catch (const std::exception& error) {
            publish_policy_status(false, "observation construction error");
            RCLCPP_ERROR(
                this->get_logger(),
                "Observation construction failed: %s",
                error.what());
            return;
        }

        if (input_tensor_values.size() != num_observations_) {
            publish_policy_status(false, "observation vector size mismatch");
            RCLCPP_ERROR(
                this->get_logger(),
                "Observation size mismatch. Expected %zu, got %zu.",
                num_observations_, input_tensor_values.size());
            return;
        }

        if (!all_finite(input_tensor_values)) {
            publish_policy_status(false, "observation vector contains non-finite value(s)");
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejected policy inference because observation vector contains non-finite value(s).");
            return;
        }

        // Publish the exact finite 45-D or 47-D vector passed to ONNX. Debug QoS is
        // best-effort/keep-last-1 so a slow echo/logger cannot back-pressure inference.
        publish_policy_debug_observation(input_tensor_values);
        if (gait_phase_enabled_) {
            publish_policy_debug_gait_phase(phase_snapshot.sample);
        }

        try {
            std::array<int64_t, 2> input_shape{
                1, static_cast<int64_t>(num_observations_)};
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                allocator_->GetInfo(),
                input_tensor_values.data(),
                input_tensor_values.size(),
                input_shape.data(),
                input_shape.size());

            const char* input_names[] = {input_name_.c_str()};
            const char* output_names[] = {output_name_.c_str()};

            auto output_tensors = session_->Run(
                Ort::RunOptions{nullptr},
                input_names, &input_tensor, 1,
                output_names, 1);

            if (output_tensors.empty() || !output_tensors.front().IsTensor()) {
                publish_policy_status(false, "ONNX returned no tensor output");
                RCLCPP_ERROR(this->get_logger(), "ONNX returned no tensor output.");
                return;
            }

            const size_t output_count = output_tensors.front()
                .GetTensorTypeAndShapeInfo().GetElementCount();
            if (output_count != num_actions_) {
                publish_policy_status(false, "ONNX output size mismatch");
                RCLCPP_ERROR(
                    this->get_logger(),
                    "ONNX output size mismatch. Expected %zu, got %zu.",
                    num_actions_, output_count);
                return;
            }

            const float* output_data = output_tensors.front().GetTensorData<float>();
            if (output_data == nullptr) {
                publish_policy_status(false, "ONNX output pointer is null");
                RCLCPP_ERROR(this->get_logger(), "ONNX output pointer is null.");
                return;
            }

            const std::vector<float> raw_actions(
                output_data, output_data + num_actions_);
            if (!all_finite(raw_actions)) {
                publish_policy_status(false, "ONNX output contains non-finite action(s)");
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejected ONNX output containing non-finite action(s).");
                return;
            }

            const auto action_result = compute_target_joint_radians(raw_actions);
            publish_policy_debug_actions(raw_actions, action_result);

            std_msgs::msg::Float64MultiArray message;
            message.data.reserve(action_result.target_clipped.size());
            for (const float value : action_result.target_clipped) {
                message.data.push_back(static_cast<double>(value));
            }
            if (policy_output_mode_ == "live") {
                if (!desired_position_pub_) {
                    throw std::runtime_error("live policy output publisher is unavailable");
                }
                desired_position_pub_->publish(message);
            } else if (policy_output_mode_ == "shadow") {
                if (!policy_shadow_desired_position_pub_) {
                    throw std::runtime_error("shadow policy output publisher is unavailable");
                }
                policy_shadow_desired_position_pub_->publish(message);
            }

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                // Keep the exact policy-semantic previous action: raw action after raw-action clipping.
                prev_actions_ = action_result.clipped_raw_actions;
            }
            if (gait_phase_enabled_) {
                // Advance only after a complete successful inference and output-path update.
                // Readiness loss and inference failures therefore freeze phase deterministically.
                advance_gait_phase_if_unchanged(phase_snapshot);
            }

            publish_policy_status(
                true,
                policy_output_mode_ == "shadow" ? "ready (shadow)" : "ready");
        } catch (const Ort::Exception& error) {
            publish_policy_status(false, "ONNX Runtime exception");
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "ONNX Runtime exception: %s", error.what());
        } catch (const std::exception& error) {
            publish_policy_status(false, "policy post-processing exception");
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Policy processing exception: %s", error.what());
        }
    }

    size_t num_observations_ = littlegreen_biped::kLegacyObservationCount;
    size_t num_actions_ = littlegreen_biped::kNumPolicyActions;
    double policy_dt_ = 0.04;

    int observation_contract_version_ = 1;
    std::string observation_contract_name_{"littlegreen_hardware_45_legacy"};
    bool gait_phase_enabled_ = false;
    double gait_phase_period_s_ = 0.0;
    std::string gait_phase_encoding_;
    std::string gait_phase_append_order_;
    std::string gait_phase_training_timebase_;
    std::string gait_phase_training_reset_semantics_;
    mutable std::mutex gait_phase_mutex_;
    GaitPhaseClock gait_phase_clock_;
    std::uint64_t gait_phase_generation_ = 0U;

    YAML::Node policy_config_;

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::AllocatorWithDefaultOptions> allocator_;
    std::string input_name_;
    std::string output_name_;

    std::vector<std::string> joint_names_;
    std::vector<int> sim_joint_indices_;
    std::vector<int> ros_joint_state_indices_;
    std::vector<int> micro_ros_array_indices_;
    std::vector<int> servo_ids_;

    std::vector<float> default_joint_positions_;
    std::vector<float> joint_lower_limits_;
    std::vector<float> joint_upper_limits_;
    std::vector<float> action_limit_lower_;
    std::vector<float> action_limit_upper_;
    int action_contract_version_ = 0;
    std::string action_contract_name_;
    std::string action_transform_;
    std::string deployment_contract_profile_;
    std::string previous_action_observation_;
    std::vector<float> action_residual_scale_rad_;
    std::vector<float> exported_action_default_rad_;
    std::vector<float> exported_action_target_lower_rad_;
    std::vector<float> exported_action_target_upper_rad_;
    std::vector<float> exported_action_nominal_lower_rad_;
    std::vector<float> exported_action_nominal_upper_rad_;
    std::vector<int> exported_action_indices_;
    std::vector<std::string> exported_sim_joint_names_;
    std::vector<float> exported_sim_default_joint_positions_;

    mutable std::mutex state_mutex_;
    std::vector<float> prev_actions_;
    std::vector<float> joint_positions_;
    std::vector<float> joint_velocities_;
    std::vector<float> base_ang_vel_;
    std::vector<float> cmd_vel_;
    std::array<float, 4> imu_orientation_wxyz_{1.0f, 0.0f, 0.0f, 0.0f};

    std::vector<bool> joint_position_seen_;
    std::vector<bool> joint_velocity_seen_;
    std::vector<SteadyClock::time_point> joint_position_update_time_;
    std::vector<SteadyClock::time_point> joint_velocity_update_time_;

    std::vector<uint32_t> joint_feedback_age_ms_;
    std::vector<bool> joint_feedback_age_seen_;
    bool have_joint_feedback_age_message_ = false;
    SteadyClock::time_point last_joint_feedback_age_update_time_{};

    bool have_valid_imu_ = false;
    bool have_valid_command_ = false;
    SteadyClock::time_point last_imu_update_time_{};
    SteadyClock::time_point last_command_update_time_{};

    std::array<float, 9> imu_to_base_matrix_{
         0.0f, 1.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f};

    double imu_timeout_sec_ = 0.050;
    double joint_state_timeout_sec_ = 0.150;
    double joint_feedback_age_topic_timeout_sec_ = 0.150;
    double joint_feedback_max_age_sec_ = 0.250;
    double command_timeout_sec_ = 0.500;
    bool require_joint_velocity_ = true;
    bool require_joint_feedback_age_ = true;
    bool zero_command_on_timeout_ = true;
    bool override_imu_ = false;
    bool use_sim_ = false;
    bool publish_policy_debug_ = true;
    std::string policy_output_mode_{"live"};
    std::string shadow_desired_position_topic_{"/policy_shadow/desired_position"};

    bool status_initialized_ = false;
    bool last_ready_state_ = false;
    std::string last_status_reason_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_shadow_desired_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr policy_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr policy_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_debug_observation_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_debug_raw_action_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_debug_clipped_raw_action_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_debug_target_unclipped_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_debug_target_clipped_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr policy_debug_saturation_mask_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_debug_gait_phase_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_gait_phase_service_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr joint_feedback_age_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_position_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_velocity_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::executors::MultiThreadedExecutor executor;
        auto node = std::make_shared<LittleGreenBipedPolicyNode>();
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& error) {
        std::cerr << "Fatal LittleGreen biped policy node error: " << error.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
