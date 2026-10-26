// deploy/include/runtime/keyboard_map.h
#pragma once

#include <algorithm>

#include "runtime/types.h"

namespace runtime {

class KeyboardMap {
 public:
  KeyboardMap(KeyboardConfig cfg, CommandLimits limits)
      : cfg_(cfg), limits_(limits) {}

  ActionSet onKey(const KeyPulse& key) {
    ActionSet actions;
    if (!key.just_pressed) return actions;

    if (key.key == cfg_.stand) {
      actions.stand = true;
    } else if (key.key == cfg_.lie) {
      actions.lie = true;
    } else if (key.key == cfg_.velocity) {
      actions.velocity = true;
    } else if (key.key == cfg_.passive) {
      actions.passive = true;
    } else if (key.key == cfg_.estop) {
      actions.estop = true;
    } else if (key.key == cfg_.rearm) {
      actions.rearm = true;
    } else if (key.key == cfg_.slow_toggle) {
      slow_ = !slow_;
    } else if (key.key == cfg_.stop) {
      setpoint_ = MotionCommand{};
    } else if (key.key == cfg_.vx_up) {
      setpoint_.vx += step(cfg_.vx_step);
    } else if (key.key == cfg_.vx_down) {
      setpoint_.vx -= step(cfg_.vx_step);
    } else if (key.key == cfg_.vy_up) {
      setpoint_.vy += step(cfg_.vy_step);
    } else if (key.key == cfg_.vy_down) {
      setpoint_.vy -= step(cfg_.vy_step);
    } else if (key.key == cfg_.wz_up) {
      setpoint_.wz += step(cfg_.wz_step);
    } else if (key.key == cfg_.wz_down) {
      setpoint_.wz -= step(cfg_.wz_step);
    }
    clampSetpoint();
    return actions;
  }

  const MotionCommand& setpoint() const { return setpoint_; }
  bool slowMode() const { return slow_; }

 private:
  float step(float base) const { return slow_ ? base * cfg_.slow_scale : base; }

  void clampSetpoint() {
    setpoint_.vx = std::clamp(setpoint_.vx, limits_.min_vx, limits_.max_vx);
    setpoint_.vy = std::clamp(setpoint_.vy, limits_.min_vy, limits_.max_vy);
    setpoint_.wz = std::clamp(setpoint_.wz, limits_.min_wz, limits_.max_wz);
  }

  KeyboardConfig cfg_;
  CommandLimits limits_;
  MotionCommand setpoint_{};
  bool slow_{false};
};

}  // namespace runtime
