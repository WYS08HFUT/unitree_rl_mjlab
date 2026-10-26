// deploy/robots/go2/src/Go2RuntimeBridge.cpp
#include "Go2RuntimeBridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

#include "FSM/FSMState.h"

namespace {

double nowSeconds() {
  using Clock = std::chrono::steady_clock;
  static const Clock::time_point start = Clock::now();
  return std::chrono::duration<double>(Clock::now() - start).count();
}

float scaleAxis(float value, float min, float max) {
  return value >= 0.0f ? value * max : value * (-min);
}

float number(const YAML::Node& node, const char* key, float fallback) {
  return node && node[key] ? node[key].as<float>() : fallback;
}

bool boolean(const YAML::Node& node, const char* key, bool fallback) {
  return node && node[key] ? node[key].as<bool>() : fallback;
}

std::string text(const YAML::Node& node, const char* key,
                 const std::string& fallback) {
  return node && node[key] ? node[key].as<std::string>() : fallback;
}

runtime::RuntimeConfig makeConfig(const YAML::Node& root,
                                  const runtime::CommandLimits& limits) {
  runtime::RuntimeConfig cfg;
  cfg.velocity_enabled = boolean(root, "velocity_enabled", false);
  cfg.limits = limits;

  const YAML::Node command = root["command"];
  cfg.command.deadzone = number(command, "deadzone", cfg.command.deadzone);
  cfg.command.accel_limit =
      number(command, "accel_limit", cfg.command.accel_limit);
  cfg.command.yaw_accel_limit =
      number(command, "yaw_accel_limit", cfg.command.yaw_accel_limit);

  const YAML::Node safety = root["safety"];
  cfg.safety.tilt_degraded =
      number(safety, "tilt_degraded", cfg.safety.tilt_degraded);
  cfg.safety.tilt_critical =
      number(safety, "tilt_critical", cfg.safety.tilt_critical);
  cfg.safety.gyro_degraded =
      number(safety, "gyro_degraded", cfg.safety.gyro_degraded);
  cfg.safety.gyro_critical =
      number(safety, "gyro_critical", cfg.safety.gyro_critical);
  cfg.safety.joint_vel_degraded =
      number(safety, "joint_vel_degraded", cfg.safety.joint_vel_degraded);
  cfg.safety.joint_vel_critical =
      number(safety, "joint_vel_critical", cfg.safety.joint_vel_critical);
  cfg.safety.action_jump_critical = number(
      safety, "action_jump_critical", cfg.safety.action_jump_critical);
  cfg.safety.recover_time =
      number(safety, "recover_time", cfg.safety.recover_time);
  cfg.safety.rearm_hold_time =
      number(safety, "rearm_hold_time", cfg.safety.rearm_hold_time);

  const YAML::Node keyboard = root["keyboard"];
  cfg.keyboard.stand = text(keyboard, "stand", cfg.keyboard.stand);
  cfg.keyboard.lie = text(keyboard, "lie", cfg.keyboard.lie);
  cfg.keyboard.velocity = text(keyboard, "velocity", cfg.keyboard.velocity);
  cfg.keyboard.passive = text(keyboard, "passive", cfg.keyboard.passive);
  cfg.keyboard.estop = text(keyboard, "estop", cfg.keyboard.estop);
  cfg.keyboard.rearm = text(keyboard, "rearm", cfg.keyboard.rearm);
  cfg.keyboard.slow_toggle =
      text(keyboard, "slow_toggle", cfg.keyboard.slow_toggle);
  cfg.keyboard.vx_step = number(keyboard, "vx_step", cfg.keyboard.vx_step);
  cfg.keyboard.vy_step = number(keyboard, "vy_step", cfg.keyboard.vy_step);
  cfg.keyboard.wz_step = number(keyboard, "wz_step", cfg.keyboard.wz_step);
  cfg.keyboard.slow_scale =
      number(keyboard, "slow_scale", cfg.keyboard.slow_scale);
  return cfg;
}

}  // namespace

Go2RuntimeBridge::Go2RuntimeBridge(const YAML::Node& cfg,
                                   const runtime::CommandLimits& limits,
                                   const GamepadAdapter::Expressions& expressions,
                                   Callbacks callbacks)
    : runtime_(makeConfig(cfg, limits)),
      limits_(limits),
      adapter_(expressions),
      callbacks_(std::move(callbacks)),
      names_{{"Passive", runtime::Behavior::kPassive},
             {"FixStand", runtime::Behavior::kStand},
             {"LieDown", runtime::Behavior::kLie},
             {"Velocity", runtime::Behavior::kVelocity}} {
  if (callbacks_.state_id) {
    for (const auto& entry : names_) {
      try {
        id_to_behavior_.emplace(callbacks_.state_id(entry.first), entry.second);
      } catch (const std::exception&) {
        // 该机器人未配置这个状态，忽略
      }
    }
  }
}

void Go2RuntimeBridge::tick() {
  if (!FSMState::lowstate) return;
  const auto& state = FSMState::lowstate;

  runtime::RobotSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    const auto& imu = state->msg_.imu_state();
    const auto& quaternion = imu.quaternion();
    const Eigen::Quaternionf q(quaternion[0], quaternion[1], quaternion[2],
                               quaternion[3]);
    const Eigen::Vector3f gravity =
        q.conjugate() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    snapshot.roll = std::atan2(-gravity.y(), -gravity.z());
    snapshot.pitch =
        std::atan2(gravity.x(), std::hypot(gravity.y(), gravity.z()));
    const auto& gyro = imu.gyroscope();
    snapshot.gyro_norm =
        std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    float max_joint_vel = 0.0f;
    const auto& motors = state->msg_.motor_state();
    for (std::size_t i = 0; i < motors.size(); ++i) {
      max_joint_vel = std::max(max_joint_vel, std::fabs(motors[i].dq()));
    }
    snapshot.max_joint_vel = max_joint_vel;
  }

  const std::string current_name =
      callbacks_.current_state_name ? callbacks_.current_state_name()
                                    : std::string();
  const std::optional<runtime::Behavior> current = behaviorFromName(current_name);
  if (current) runtime_.setCurrentBehavior(*current);

  runtime::KeyPulse key;
  if (FSMState::keyboard) {
    key.key = FSMState::keyboard->key();
    key.just_pressed = FSMState::keyboard->on_pressed;
  }

  const runtime::ActionSet pad_actions = adapter_.actions(state->joystick);
  runtime::MotionCommand pad_motion;
  pad_motion.vx =
      scaleAxis(state->joystick.ly(), limits_.min_vx, limits_.max_vx);
  pad_motion.vy =
      scaleAxis(-state->joystick.lx(), limits_.min_vy, limits_.max_vy);
  pad_motion.wz =
      scaleAxis(-state->joystick.rx(), limits_.min_wz, limits_.max_wz);

  const runtime::TickOutput output =
      runtime_.tick(nowSeconds(), snapshot, pad_actions, pad_motion,
                    !state->isJoystickTimeout(), key);

  if (current && output.behavior != *current && callbacks_.request_state) {
    const std::optional<int> id = stateId(output.behavior);
    if (id) callbacks_.request_state(*id);
  }
}

bool Go2RuntimeBridge::allowTransition(int fsm_id) const {
  const auto it = id_to_behavior_.find(fsm_id);
  if (it == id_to_behavior_.end()) return true;
  return runtime_.allowTransition(it->second);
}

std::optional<runtime::Behavior> Go2RuntimeBridge::behaviorFromName(
    const std::string& name) const {
  const auto it = names_.find(name);
  if (it == names_.end()) return std::nullopt;
  return it->second;
}

std::optional<int> Go2RuntimeBridge::stateId(runtime::Behavior behavior) const {
  for (const auto& entry : id_to_behavior_) {
    if (entry.second == behavior) return entry.first;
  }
  return std::nullopt;
}
