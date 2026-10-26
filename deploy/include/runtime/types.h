// deploy/include/runtime/types.h
#pragma once

#include <string>

namespace runtime {

enum class Behavior { kPassive, kStand, kLie, kVelocity };
enum class SafetyLevel { kOk, kDegraded, kCritical };

struct MotionCommand {
  float vx{0.0f};
  float vy{0.0f};
  float wz{0.0f};
};

struct CommandLimits {
  float min_vx{-0.5f};
  float max_vx{1.0f};
  float min_vy{-0.5f};
  float max_vy{0.5f};
  float min_wz{-1.0f};
  float max_wz{1.0f};
};

struct RobotSnapshot {
  float roll{0.0f};
  float pitch{0.0f};
  float gyro_norm{0.0f};
  float max_joint_vel{0.0f};
  bool action_finite{true};
  bool action_jump{false};
};

struct ActionSet {
  bool stand{false};
  bool lie{false};
  bool velocity{false};
  bool passive{false};
  bool estop{false};
  bool rearm{false};
};

struct KeyPulse {
  std::string key;
  bool just_pressed{false};
};

struct KeyboardConfig {
  std::string stand{"1"};
  std::string lie{"2"};
  std::string velocity{"3"};
  std::string passive{"0"};
  std::string estop{"x"};
  std::string rearm{"r"};
  std::string vx_up{"w"};
  std::string vx_down{"s"};
  std::string vy_up{"a"};
  std::string vy_down{"d"};
  std::string wz_up{"q"};
  std::string wz_down{"e"};
  std::string stop{" "};
  std::string slow_toggle{"z"};
  float vx_step{0.1f};
  float vy_step{0.1f};
  float wz_step{0.2f};
  float slow_scale{0.4f};
};

struct CommandMuxConfig {
  float deadzone{0.05f};
  float accel_limit{2.0f};
  float yaw_accel_limit{4.0f};
};

struct SafetyConfig {
  float tilt_degraded{0.35f};
  float tilt_critical{0.60f};
  float gyro_degraded{3.5f};
  float gyro_critical{6.0f};
  float joint_vel_degraded{30.0f};
  float joint_vel_critical{50.0f};
  float action_jump_critical{1.5f};
  float recover_time{0.5f};
  float rearm_hold_time{0.5f};
};

struct TickOutput {
  Behavior behavior{Behavior::kPassive};
  MotionCommand command{};
  SafetyLevel safety{SafetyLevel::kOk};
  bool latched{false};
};

struct RuntimeConfig {
  bool velocity_enabled{true};
  CommandLimits limits{};
  KeyboardConfig keyboard{};
  CommandMuxConfig command{};
  SafetyConfig safety{};
};

}  // namespace runtime
