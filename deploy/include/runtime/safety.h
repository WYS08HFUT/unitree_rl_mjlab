// deploy/include/runtime/safety.h
#pragma once

#include <algorithm>
#include <cmath>

#include "runtime/types.h"

namespace runtime {

class SafetySupervisor {
 public:
  explicit SafetySupervisor(SafetyConfig cfg) : cfg_(cfg) {}

  SafetyLevel update(const RobotSnapshot& robot, bool estop, double now) {
    const SafetyLevel raw = rawLevel(robot, estop);

    if (raw == SafetyLevel::kCritical) {
      latched_ = true;
      level_ = SafetyLevel::kCritical;
      ok_since_ = kNoTime;
      return level_;
    }

    if (latched_) {
      if (raw == SafetyLevel::kOk) {
        if (ok_since_ == kNoTime) ok_since_ = now;
      } else {
        ok_since_ = kNoTime;
      }
      level_ = SafetyLevel::kCritical;
      return level_;
    }

    if (raw == SafetyLevel::kDegraded) {
      level_ = SafetyLevel::kDegraded;
      ok_since_ = kNoTime;
      return level_;
    }

    if (ok_since_ == kNoTime) ok_since_ = now;
    if (level_ == SafetyLevel::kDegraded &&
        now - ok_since_ < cfg_.recover_time) {
      return level_;
    }
    level_ = SafetyLevel::kOk;
    return level_;
  }

  bool tryRearm(double now) {
    if (!latched_ || ok_since_ == kNoTime) return false;
    if (now - ok_since_ < cfg_.rearm_hold_time) return false;
    latched_ = false;
    level_ = SafetyLevel::kOk;
    return true;
  }

  bool latched() const { return latched_; }
  SafetyLevel level() const { return level_; }

 private:
  static constexpr double kNoTime = -1e9;

  SafetyLevel rawLevel(const RobotSnapshot& robot, bool estop) const {
    if (estop || !robot.action_finite || robot.action_jump) {
      return SafetyLevel::kCritical;
    }
    if (!std::isfinite(robot.roll) || !std::isfinite(robot.pitch) ||
        !std::isfinite(robot.gyro_norm) ||
        !std::isfinite(robot.max_joint_vel)) {
      return SafetyLevel::kCritical;
    }
    const float tilt = std::max(std::fabs(robot.roll), std::fabs(robot.pitch));
    if (tilt > cfg_.tilt_critical || robot.gyro_norm > cfg_.gyro_critical ||
        robot.max_joint_vel > cfg_.joint_vel_critical) {
      return SafetyLevel::kCritical;
    }
    if (tilt > cfg_.tilt_degraded || robot.gyro_norm > cfg_.gyro_degraded ||
        robot.max_joint_vel > cfg_.joint_vel_degraded) {
      return SafetyLevel::kDegraded;
    }
    return SafetyLevel::kOk;
  }

  SafetyConfig cfg_;
  SafetyLevel level_{SafetyLevel::kOk};
  bool latched_{false};
  double ok_since_{kNoTime};
};

}  // namespace runtime
