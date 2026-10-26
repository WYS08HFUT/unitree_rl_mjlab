// deploy/include/runtime/behavior.h
#pragma once

#include "runtime/safety.h"
#include "runtime/types.h"

namespace runtime {

class BehaviorRouter {
 public:
  Behavior update(const ActionSet& actions, SafetySupervisor& safety,
                  Behavior current, double now, bool velocity_enabled) {
    if (safety.latched()) {
      if (actions.rearm && current == Behavior::kPassive) {
        safety.tryRearm(now);
      }
      return Behavior::kPassive;
    }

    if (actions.estop) return Behavior::kPassive;
    if (actions.passive) return Behavior::kPassive;
    if (actions.lie) return Behavior::kLie;
    if (actions.stand) return Behavior::kStand;
    if (actions.velocity && velocity_enabled) return Behavior::kVelocity;
    return current;
  }
};

}  // namespace runtime
