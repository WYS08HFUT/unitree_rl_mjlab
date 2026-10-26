// deploy/include/runtime/command_mux.h
#pragma once

#include <algorithm>
#include <cmath>

#include "runtime/types.h"

namespace runtime {

class CommandMux {
 public:
  CommandMux(CommandMuxConfig cfg, CommandLimits limits)
      : cfg_(cfg), limits_(limits) {}

  MotionCommand update(const MotionCommand& manual, bool active,
                       SafetyLevel safety, double dt) {
    if (!active || safety != SafetyLevel::kOk) {
      target_ = MotionCommand{};
      output_ = MotionCommand{};
      return output_;
    }

    target_.vx = std::clamp(deadzone(manual.vx), limits_.min_vx, limits_.max_vx);
    target_.vy = std::clamp(deadzone(manual.vy), limits_.min_vy, limits_.max_vy);
    target_.wz = std::clamp(deadzone(manual.wz), limits_.min_wz, limits_.max_wz);

    output_.vx = slew(output_.vx, target_.vx, cfg_.accel_limit * dt);
    output_.vy = slew(output_.vy, target_.vy, cfg_.accel_limit * dt);
    output_.wz = slew(output_.wz, target_.wz, cfg_.yaw_accel_limit * dt);
    return output_;
  }

  const MotionCommand& target() const { return target_; }
  const MotionCommand& output() const { return output_; }

 private:
  float deadzone(float v) const {
    return std::fabs(v) < cfg_.deadzone ? 0.0f : v;
  }

  static float slew(float current, float target, float max_delta) {
    const float delta = target - current;
    if (std::fabs(delta) <= max_delta) return target;
    return current + (delta > 0.0f ? max_delta : -max_delta);
  }

  CommandMuxConfig cfg_;
  CommandLimits limits_;
  MotionCommand target_{};
  MotionCommand output_{};
};

}  // namespace runtime
