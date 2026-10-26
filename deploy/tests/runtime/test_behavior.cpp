// deploy/tests/runtime/test_behavior.cpp
#include "runtime/behavior.h"
#include "test_framework.h"

using namespace runtime;

void test_behavior() {
  const SafetyConfig cfg;
  SafetySupervisor supervisor(cfg);
  BehaviorRouter router;
  RobotSnapshot good;
  supervisor.update(good, false, 0.0);

  ActionSet actions;
  actions.stand = true;
  CHECK(router.update(actions, supervisor, Behavior::kPassive, 0.1, true) ==
        Behavior::kStand);

  actions = ActionSet{};
  actions.lie = true;
  CHECK(router.update(actions, supervisor, Behavior::kStand, 0.2, true) ==
        Behavior::kLie);

  actions = ActionSet{};
  actions.passive = true;
  CHECK(router.update(actions, supervisor, Behavior::kStand, 0.3, true) ==
        Behavior::kPassive);

  actions = ActionSet{};
  actions.velocity = true;
  CHECK(router.update(actions, supervisor, Behavior::kPassive, 0.4, true) ==
        Behavior::kVelocity);
  CHECK(router.update(actions, supervisor, Behavior::kPassive, 0.5, false) ==
        Behavior::kPassive);

  RobotSnapshot hard;
  hard.pitch = 0.7f;
  supervisor.update(hard, false, 1.0);

  actions = ActionSet{};
  actions.stand = true;
  supervisor.update(good, false, 1.1);
  CHECK(router.update(actions, supervisor, Behavior::kPassive, 1.1, true) ==
        Behavior::kPassive);

  actions = ActionSet{};
  actions.rearm = true;
  CHECK(router.update(actions, supervisor, Behavior::kPassive, 1.2, true) ==
        Behavior::kPassive);
  CHECK(supervisor.latched());

  supervisor.update(good, false, 1.5);
  supervisor.update(good, false, 1.65);
  CHECK(router.update(actions, supervisor, Behavior::kPassive, 1.65, true) ==
        Behavior::kPassive);
  CHECK(!supervisor.latched());

  // ---- Added assertions: priority order, latch gating, velocity flag, rearm ----

  // [1] No action + not latched holds `current` (the fall-through is not kPassive).
  {
    BehaviorRouter idle;
    SafetySupervisor safe(cfg);
    safe.update(good, false, 0.0);
    CHECK(idle.update(ActionSet{}, safe, Behavior::kStand, 0.1, true) ==
          Behavior::kStand);
    CHECK(idle.update(ActionSet{}, safe, Behavior::kVelocity, 0.2, true) ==
          Behavior::kVelocity);
  }

  // [2] estop outranks lie/stand/velocity and lands on kPassive.
  {
    BehaviorRouter estop_router;
    SafetySupervisor fresh(cfg);
    fresh.update(good, false, 0.0);

    ActionSet only_estop;
    only_estop.estop = true;
    CHECK(estop_router.update(only_estop, fresh, Behavior::kVelocity, 0.1, true) ==
          Behavior::kPassive);

    ActionSet estop_with_others;
    estop_with_others.estop = true;
    estop_with_others.stand = true;
    estop_with_others.lie = true;
    estop_with_others.velocity = true;
    CHECK(estop_router.update(estop_with_others, fresh, Behavior::kVelocity, 0.2,
                              true) == Behavior::kPassive);
    CHECK(!fresh.latched());  // the router does not latch on its own
  }

  // [3] While latched every non-rearm action is ignored and the output is
  //     kPassive, never `current` (which here is a non-passive behavior).
  {
    BehaviorRouter latched_router;
    SafetySupervisor latched(cfg);
    RobotSnapshot critical;
    critical.roll = 1.0f;
    CHECK(latched.update(critical, false, 0.0) == SafetyLevel::kCritical);
    CHECK(latched.latched());

    ActionSet vel;
    vel.velocity = true;
    CHECK(latched_router.update(vel, latched, Behavior::kVelocity, 0.1, true) ==
          Behavior::kPassive);

    ActionSet lie;
    lie.lie = true;
    CHECK(latched_router.update(lie, latched, Behavior::kVelocity, 0.2, true) ==
          Behavior::kPassive);

    ActionSet up;
    up.stand = true;
    CHECK(latched_router.update(up, latched, Behavior::kPassive, 0.3, true) ==
          Behavior::kPassive);

    // rearm without any OK streak: still latched, still kPassive, and the latch
    // must not be cleared by the router on its own.
    ActionSet rearm_and_stand;
    rearm_and_stand.rearm = true;
    rearm_and_stand.stand = true;
    CHECK(latched_router.update(rearm_and_stand, latched, Behavior::kPassive, 0.4,
                               true) == Behavior::kPassive);
    CHECK(latched.latched());
  }

  // [4] Rearm is routed through the supervisor's hold time: before the hold only
  //     kPassive is reported; at exactly rearm_hold_time the latch clears.
  {
    BehaviorRouter rearm_router;
    SafetySupervisor rearm_sup(cfg);
    RobotSnapshot critical;
    critical.pitch = -0.8f;
    CHECK(rearm_sup.update(critical, false, 0.0) == SafetyLevel::kCritical);
    // The first OK tick starts the streak; while latched update() keeps kCritical.
    CHECK(rearm_sup.update(good, false, 1.0) == SafetyLevel::kCritical);

    ActionSet rearm_action;
    rearm_action.rearm = true;
    // Zero hold elapsed: tryRearm(now=1.0) must fail, and a non-passive `current`
    // must not leak through the latch.
    CHECK(rearm_router.update(rearm_action, rearm_sup, Behavior::kStand, 1.0, true) ==
          Behavior::kPassive);
    CHECK(rearm_sup.latched());

    // 0.2 s of continuous OK < rearm_hold_time (0.5 s): hold not met.
    ActionSet rearm_and_stand = rearm_action;
    rearm_and_stand.stand = true;
    CHECK(rearm_router.update(rearm_and_stand, rearm_sup, Behavior::kPassive, 1.2,
                              true) == Behavior::kPassive);
    CHECK(rearm_sup.latched());

    // Exactly rearm_hold_time of continuous OK re-arms; still reports kPassive.
    CHECK(rearm_router.update(rearm_action, rearm_sup, Behavior::kPassive, 1.5,
                              true) == Behavior::kPassive);
    CHECK(!rearm_sup.latched());
    CHECK(rearm_sup.level() == SafetyLevel::kOk);

    // Once unlatched the still-held rearm flag is ignored: a stand action wins.
    ActionSet stand_after_rearm = rearm_action;
    stand_after_rearm.stand = true;
    CHECK(rearm_router.update(stand_after_rearm, rearm_sup, Behavior::kPassive, 1.6,
                              true) == Behavior::kStand);
  }

  // [5] A rearm while not latched is a no-op (does not force kPassive and leaves
  //     the supervisor level untouched).
  {
    BehaviorRouter unlocked_router;
    SafetySupervisor unlocked(cfg);
    unlocked.update(good, false, 0.0);
    ActionSet rearm;
    rearm.rearm = true;
    CHECK(unlocked_router.update(rearm, unlocked, Behavior::kStand, 0.1, true) ==
          Behavior::kStand);
    CHECK(!unlocked.latched());
    CHECK(unlocked.level() == SafetyLevel::kOk);
  }

  // [6] velocity_enabled=false drops the velocity action and falls through to
  //     `current` (not kPassive); lie/stand still outrank velocity either way.
  {
    BehaviorRouter no_vel;
    SafetySupervisor fresh(cfg);
    fresh.update(good, false, 0.0);

    ActionSet vel;
    vel.velocity = true;
    CHECK(no_vel.update(vel, fresh, Behavior::kStand, 0.1, false) ==
          Behavior::kStand);
    CHECK(no_vel.update(vel, fresh, Behavior::kVelocity, 0.2, false) ==
          Behavior::kVelocity);

    ActionSet stand_and_vel;
    stand_and_vel.stand = true;
    stand_and_vel.velocity = true;
    CHECK(no_vel.update(stand_and_vel, fresh, Behavior::kPassive, 0.3, false) ==
          Behavior::kStand);
    CHECK(no_vel.update(stand_and_vel, fresh, Behavior::kPassive, 0.4, true) ==
          Behavior::kStand);
  }

  // [7] Full non-latched precedence: passive > lie > stand > velocity.
  {
    BehaviorRouter prio;
    SafetySupervisor fresh(cfg);
    fresh.update(good, false, 0.0);

    ActionSet lie_stand_vel;
    lie_stand_vel.lie = true;
    lie_stand_vel.stand = true;
    lie_stand_vel.velocity = true;
    CHECK(prio.update(lie_stand_vel, fresh, Behavior::kPassive, 0.1, true) ==
          Behavior::kLie);

    ActionSet passive_lie;
    passive_lie.passive = true;
    passive_lie.lie = true;
    CHECK(prio.update(passive_lie, fresh, Behavior::kLie, 0.2, true) ==
          Behavior::kPassive);

    ActionSet passive_estop;  // estop and passive both map to kPassive
    passive_estop.passive = true;
    passive_estop.estop = true;
    CHECK(prio.update(passive_estop, fresh, Behavior::kStand, 0.3, true) ==
          Behavior::kPassive);
  }

  // [8] The latch clears only on request: with the OK hold long satisfied, a
  //     stand action must leave it latched (no auto-rearm).
  {
    BehaviorRouter hold_router;
    SafetySupervisor hold_sup(cfg);
    RobotSnapshot critical;
    critical.roll = -1.0f;
    CHECK(hold_sup.update(critical, false, 0.0) == SafetyLevel::kCritical);
    CHECK(hold_sup.update(good, false, 1.0) == SafetyLevel::kCritical);
    CHECK(hold_sup.update(good, false, 2.0) == SafetyLevel::kCritical);  // 1.0 s OK

    ActionSet stand;
    stand.stand = true;
    CHECK(hold_router.update(stand, hold_sup, Behavior::kPassive, 2.0, true) ==
          Behavior::kPassive);
    CHECK(hold_sup.latched());

    // The same supervisor re-arms immediately once rearm is actually requested.
    ActionSet rearm;
    rearm.rearm = true;
    CHECK(hold_router.update(rearm, hold_sup, Behavior::kPassive, 2.1, true) ==
          Behavior::kPassive);
    CHECK(!hold_sup.latched());
  }
}
