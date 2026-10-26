// deploy/robots/go2/src/Go2RuntimeBridge.cpp
// SDK dependencies: unitree/dds_wrapper/robots/go2/go2.h (via FSM/Types.h),
// unitree/dds_wrapper/common/unitree_joystick.hpp (via unitree_joystick_dsl.hpp).
#include "Go2RuntimeBridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "FSM/FSMState.h"
#include "Go2RuntimeConfig.h"

namespace {

double nowSeconds() {
  using Clock = std::chrono::steady_clock;
  static const Clock::time_point start = Clock::now();
  return std::chrono::duration<double>(Clock::now() - start).count();
}

float scaleAxis(float value, float min, float max) {
  return value >= 0.0f ? value * max : value * (-min);
}

}  // namespace

Go2RuntimeBridge::Go2RuntimeBridge(const YAML::Node& cfg,
                                   const runtime::CommandLimits& limits,
                                   const GamepadAdapter::Expressions& expressions,
                                   Callbacks callbacks)
    : runtime_(go2::makeRuntimeConfig(cfg, limits)),
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
        // Unconfigured state: disable that behavior (Velocity is expected absent).
        if (entry.first != "Velocity") {
          spdlog::warn("Runtime: FSM state '{}' not configured; behavior disabled",
                       entry.first);
        }
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
    // A degenerate quaternion yields a non-finite tilt so safety fails closed.
    const float quat_norm = q.norm();
    if (!std::isfinite(quat_norm) || quat_norm < 0.5f) {
      snapshot.roll = std::numeric_limits<float>::quiet_NaN();
      snapshot.pitch = snapshot.roll;
    } else {
      const Eigen::Vector3f gravity =
          q.conjugate() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
      snapshot.roll = std::atan2(-gravity.y(), -gravity.z());
      snapshot.pitch =
          std::atan2(gravity.x(), std::hypot(gravity.y(), gravity.z()));
    }
    const auto& gyro = imu.gyroscope();
    snapshot.gyro_norm = std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] +
                                   gyro[2] * gyro[2]);

    // Only 12 Go2 joints matter; a non-finite dq poisons the max into NaN.
    constexpr std::size_t kJointCount = 12;
    const auto& motors = state->msg_.motor_state();
    const std::size_t motor_count = std::min(motors.size(), kJointCount);
    float max_joint_vel = 0.0f;
    for (std::size_t i = 0; i < motor_count; ++i) {
      const float dq = motors[i].dq();
      if (!std::isfinite(dq)) {
        max_joint_vel = std::numeric_limits<float>::quiet_NaN();
        break;
      }
      max_joint_vel = std::max(max_joint_vel, std::fabs(dq));
    }
    snapshot.max_joint_vel = max_joint_vel;
  }
  const std::string current_name =
      callbacks_.current_state_name ? callbacks_.current_state_name() : "";
  const std::optional<runtime::Behavior> current = behaviorFromName(current_name);
  if (current) runtime_.setCurrentBehavior(*current);

  runtime::KeyPulse key;
  if (FSMState::keyboard) {
    key = runtime::KeyPulse{FSMState::keyboard->key(),
                            FSMState::keyboard->on_pressed};
  }
  const runtime::ActionSet pad_actions = adapter_.actions(state->joystick);
  const runtime::MotionCommand pad_motion{
      scaleAxis(state->joystick.ly(), limits_.min_vx, limits_.max_vx),
      scaleAxis(-state->joystick.lx(), limits_.min_vy, limits_.max_vy),
      scaleAxis(-state->joystick.rx(), limits_.min_wz, limits_.max_wz)};

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
  if (it == id_to_behavior_.end()) {
    // Unknown ids cannot be proven safe: fail closed like a non-passive target.
    return runtime_.allowTransition(runtime::Behavior::kStand);
  }
  return runtime_.allowTransition(it->second);
}

std::optional<runtime::Behavior> Go2RuntimeBridge::behaviorFromName(
    const std::string& name) const {
  const auto it = names_.find(name);
  return it == names_.end() ? std::nullopt : std::optional(it->second);
}

std::optional<int> Go2RuntimeBridge::stateId(runtime::Behavior behavior) const {
  for (const auto& entry : id_to_behavior_) {
    if (entry.second == behavior) return entry.first;
  }
  return std::nullopt;
}
