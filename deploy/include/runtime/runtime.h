// deploy/include/runtime/runtime.h
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "runtime/behavior.h"
#include "runtime/command_mux.h"
#include "runtime/keyboard_map.h"
#include "runtime/safety.h"
#include "runtime/types.h"

namespace runtime {

class Runtime {
 public:
  explicit Runtime(RuntimeConfig cfg)
      : cfg_(cfg),
        keyboard_(cfg.keyboard, cfg.limits),
        mux_(cfg.command, cfg.limits),
        safety_(cfg.safety) {}

  void setCurrentBehavior(Behavior behavior) { behavior_ = behavior; }
  Behavior currentBehavior() const { return behavior_; }

  void notifyAction(const float* q, std::size_t n, double /*now*/) {
    action_finite_ = true;
    action_jump_ = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (!std::isfinite(q[i])) action_finite_ = false;
    }
    if (previous_action_.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        if (std::fabs(q[i] - previous_action_[i]) >
            cfg_.safety.action_jump_critical) {
          action_jump_ = true;
        }
      }
    }
    previous_action_.assign(q, q + n);
  }

  TickOutput tick(double now, const RobotSnapshot& robot_in,
                  const ActionSet& gamepad_actions,
                  const MotionCommand& gamepad_motion, bool gamepad_fresh,
                  const KeyPulse& key) {
    const double dt =
        last_tick_ == kNoTime ? 0.0 : std::clamp(now - last_tick_, 0.0, 0.05);
    last_tick_ = now;

    ActionSet actions = keyboard_.onKey(key);
    actions.stand = actions.stand || gamepad_actions.stand;
    actions.lie = actions.lie || gamepad_actions.lie;
    actions.velocity = actions.velocity || gamepad_actions.velocity;
    actions.passive = actions.passive || gamepad_actions.passive;
    actions.estop = actions.estop || gamepad_actions.estop;
    actions.rearm = actions.rearm || gamepad_actions.rearm;

    RobotSnapshot robot = robot_in;
    robot.action_finite = robot.action_finite && action_finite_;
    robot.action_jump = robot.action_jump || action_jump_;

    const SafetyLevel safety = safety_.update(robot, actions.estop, now);
    behavior_ =
        router_.update(actions, safety_, behavior_, now, cfg_.velocity_enabled);

    const bool pad_active =
        gamepad_fresh &&
        (std::fabs(gamepad_motion.vx) + std::fabs(gamepad_motion.vy) +
             std::fabs(gamepad_motion.wz) >
         1e-3f);
    const MotionCommand manual =
        pad_active ? gamepad_motion : keyboard_.setpoint();

    const MotionCommand command = mux_.update(
        manual, behavior_ == Behavior::kVelocity && safety == SafetyLevel::kOk,
        safety, dt);

    TickOutput out;
    out.behavior = behavior_;
    out.command = command;
    out.safety = safety;
    out.latched = safety_.latched();
    return out;
  }

  bool allowTransition(Behavior target) const {
    if (target == Behavior::kPassive) return true;
    if (safety_.latched() || safety_.level() == SafetyLevel::kCritical) {
      return false;
    }
    return true;
  }

  const MotionCommand& command() const { return mux_.output(); }

 private:
  static constexpr double kNoTime = -1e9;

  RuntimeConfig cfg_;
  KeyboardMap keyboard_;
  CommandMux mux_;
  SafetySupervisor safety_;
  BehaviorRouter router_;
  Behavior behavior_{Behavior::kPassive};
  double last_tick_{kNoTime};
  bool action_finite_{true};
  bool action_jump_{false};
  std::vector<float> previous_action_;
};

}  // namespace runtime
