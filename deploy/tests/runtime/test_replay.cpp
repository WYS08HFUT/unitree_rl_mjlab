// deploy/tests/runtime/test_replay.cpp
#include <cmath>

#include "runtime/runtime.h"
#include "test_framework.h"

using namespace runtime;

namespace {
KeyPulse tap(const char* key) { return KeyPulse{key, true}; }
}

void test_replay() {
  RuntimeConfig cfg;
  Runtime rt(cfg);
  const RobotSnapshot good;

  CHECK(rt.tick(0.0, good, ActionSet{}, MotionCommand{}, true, tap("1")).behavior ==
        Behavior::kStand);
  CHECK(rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, tap("3")).behavior ==
        Behavior::kVelocity);

  rt.tick(0.04, good, ActionSet{}, MotionCommand{}, true, tap("w"));
  TickOutput out = rt.tick(0.06, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
  CHECK(out.command.vx > 0.0f);

  RobotSnapshot tilt;
  tilt.roll = 0.4f;
  out = rt.tick(0.08, tilt, ActionSet{}, MotionCommand{}, true, KeyPulse{});
  CHECK(out.safety == SafetyLevel::kDegraded);
  CHECK(out.command.vx == 0.0f);

  out = rt.tick(2.0, good, ActionSet{}, MotionCommand{}, true, tap("x"));
  CHECK(out.safety == SafetyLevel::kCritical);
  CHECK(out.latched);
  CHECK(out.behavior == Behavior::kPassive);
  CHECK(!rt.allowTransition(Behavior::kStand));
  CHECK(rt.allowTransition(Behavior::kPassive));

  out = rt.tick(2.1, good, ActionSet{}, MotionCommand{}, true, tap("r"));
  CHECK(out.latched);

  out = rt.tick(2.7, good, ActionSet{}, MotionCommand{}, true, tap("r"));
  CHECK(!out.latched);

  CHECK(rt.tick(2.8, good, ActionSet{}, MotionCommand{}, true, tap("1")).behavior ==
        Behavior::kStand);
  CHECK(rt.tick(3.0, good, ActionSet{}, MotionCommand{}, true, tap("2")).behavior ==
        Behavior::kLie);
  CHECK(rt.tick(3.2, good, ActionSet{}, MotionCommand{}, true, tap("1")).behavior ==
        Behavior::kStand);

  RuntimeConfig no_velocity;
  no_velocity.velocity_enabled = false;
  Runtime rt2(no_velocity);
  CHECK(rt2.tick(0.0, good, ActionSet{}, MotionCommand{}, true, tap("3")).behavior ==
        Behavior::kPassive);

  RuntimeConfig jump_cfg;
  Runtime rt3(jump_cfg);
  float first[2] = {0.0f, 0.0f};
  float second[2] = {2.0f, 0.0f};
  rt3.notifyAction(first, 2, 0.0);
  rt3.notifyAction(second, 2, 0.01);
  CHECK(rt3.tick(0.02, good, ActionSet{}, MotionCommand{}, true, KeyPulse{}).safety ==
        SafetyLevel::kCritical);

  // ---- Added assertions: discriminating coverage beyond the brief ----

  // [1] The latch gate reopens once the rearm at 2.7 cleared it: a gate that
  //     stays closed for every non-passive target after any latch is wrong.
  CHECK(rt.allowTransition(Behavior::kStand));
  CHECK(rt.allowTransition(Behavior::kLie));

  // [2] Manual source arbitration. The keyboard setpoint is +0.1 (one `w` tap),
  //     so any output above 0.1 proves the fresh gamepad axes won; falling back
  //     to exactly 0.1 proves the keyboard won for stale / near-zero axes.
  {
    Runtime source_rt(RuntimeConfig{});
    CHECK(source_rt.tick(0.00, good, ActionSet{}, MotionCommand{}, true, tap("3"))
              .behavior == Behavior::kVelocity);
    source_rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, tap("w"));

    MotionCommand pad;
    pad.vx = 0.9f;
    TickOutput pad_out =
        source_rt.tick(0.04, good, ActionSet{}, pad, true, KeyPulse{});
    CHECK(std::fabs(pad_out.command.vx - 0.08f) < 1e-6f);  // 0.04 + 2.0*0.02

    pad_out = source_rt.tick(0.06, good, ActionSet{}, pad, true, KeyPulse{});
    CHECK(pad_out.command.vx > 0.1f);  // keyboard alone can never exceed 0.1

    const TickOutput stale =
        source_rt.tick(0.08, good, ActionSet{}, pad, false, KeyPulse{});
    CHECK(std::fabs(stale.command.vx - 0.1f) < 1e-6f);  // keyboard setpoint

    MotionCommand jitter;
    jitter.vx = 0.0005f;  // below the 1e-3 gamepad activity threshold
    const TickOutput tiny =
        source_rt.tick(0.10, good, ActionSet{}, jitter, true, KeyPulse{});
    CHECK(std::fabs(tiny.command.vx - 0.1f) < 1e-6f);  // still keyboard
  }

  // [3] Gamepad actions are merged with (not replaced by) keyboard actions:
  //     a pad-only stand switches behavior, a pad-only estop latches.
  {
    Runtime pad_action_rt(RuntimeConfig{});
    ActionSet pad_stand;
    pad_stand.stand = true;
    CHECK(pad_action_rt.tick(0.00, good, pad_stand, MotionCommand{}, true,
                             KeyPulse{})
              .behavior == Behavior::kStand);

    ActionSet pad_estop;
    pad_estop.estop = true;
    const TickOutput stopped = pad_action_rt.tick(0.02, good, pad_estop,
                                                 MotionCommand{}, true, KeyPulse{});
    CHECK(stopped.safety == SafetyLevel::kCritical);
    CHECK(stopped.latched);
    CHECK(stopped.behavior == Behavior::kPassive);
    CHECK(!pad_action_rt.allowTransition(Behavior::kStand));
  }

  // [4] The mux is active only in kVelocity: the keyboard setpoint is already
  //     0.1 while standing, yet the stand command stays exactly zero.
  {
    Runtime gate_rt(RuntimeConfig{});
    CHECK(gate_rt.tick(0.00, good, ActionSet{}, MotionCommand{}, true, tap("1"))
              .behavior == Behavior::kStand);

    const TickOutput stand_out =
        gate_rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, tap("w"));
    CHECK(stand_out.behavior == Behavior::kStand);
    CHECK(stand_out.command.vx == 0.0f);
    CHECK(stand_out.command.vy == 0.0f);
    CHECK(stand_out.command.wz == 0.0f);

    const TickOutput vel_out =
        gate_rt.tick(0.04, good, ActionSet{}, MotionCommand{}, true, tap("3"));
    CHECK(vel_out.behavior == Behavior::kVelocity);
    CHECK(vel_out.command.vx > 0.0f);
  }

  // [5] Safety gating is dynamic, not sticky: DEGRADED zeroes the command but
  //     keeps the behavior and does not latch; after 0.5 s of kOk the command
  //     resumes.
  {
    Runtime recover_rt(RuntimeConfig{});
    recover_rt.tick(0.00, good, ActionSet{}, MotionCommand{}, true, tap("3"));
    recover_rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, tap("w"));
    const TickOutput rolling =
        recover_rt.tick(0.04, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(rolling.command.vx > 0.0f);

    RobotSnapshot degraded_robot;
    degraded_robot.roll = 0.4f;
    const TickOutput degraded = recover_rt.tick(0.06, degraded_robot, ActionSet{},
                                               MotionCommand{}, true, KeyPulse{});
    CHECK(degraded.safety == SafetyLevel::kDegraded);
    CHECK(!degraded.latched);
    CHECK(degraded.behavior == Behavior::kVelocity);  // DEGRADED keeps behavior
    CHECK(degraded.command.vx == 0.0f);

    const TickOutput still_degraded = recover_rt.tick(
        0.08, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(still_degraded.safety == SafetyLevel::kDegraded);
    CHECK(still_degraded.command.vx == 0.0f);

    const TickOutput recovered =
        recover_rt.tick(0.6, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(recovered.safety == SafetyLevel::kOk);
    CHECK(!recovered.latched);
    CHECK(recovered.command.vx > 0.0f);  // mux active again
  }

  // [6] dt is clamped into [0, 0.05]. With accel_limit 0.5 and a target of 0.1
  //     the first tick integrates 0.5*0.02 = 0.01; a 9.98 s gap must integrate
  //     a clamped 0.5*0.05 = 0.025, not reach the 0.1 target, and a backwards
  //     timestamp must integrate zero.
  {
    RuntimeConfig slow_cfg;
    slow_cfg.command.accel_limit = 0.5f;
    Runtime dt_rt(slow_cfg);
    dt_rt.tick(0.00, good, ActionSet{}, MotionCommand{}, true, tap("3"));
    dt_rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, tap("w"));

    const TickOutput gap =
        dt_rt.tick(10.0, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(std::fabs(gap.command.vx - 0.035f) < 1e-6f);  // 0.01 + 0.025
    CHECK(gap.command.vx < 0.1f);                       // did not reach target

    const TickOutput backwards =
        dt_rt.tick(9.9, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(std::fabs(backwards.command.vx - 0.035f) < 1e-6f);  // dt clamped to 0
  }

  // [7] notifyAction flags: NaN latches, and the jump flag is recomputed per
  //     call (a smooth follow-up action clears it instead of staying latched).
  {
    Runtime nan_rt(RuntimeConfig{});
    const float nan_action[2] = {0.0f, std::nanf("")};
    nan_rt.notifyAction(nan_action, 2, 0.0);
    const TickOutput nan_out =
        nan_rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(nan_out.safety == SafetyLevel::kCritical);
    CHECK(nan_out.latched);

    Runtime clear_rt(RuntimeConfig{});
    const float smooth_first[2] = {0.0f, 0.0f};
    const float smooth_jump[2] = {2.0f, 0.0f};  // |Δ| = 2.0 > 1.5
    const float smooth_next[2] = {2.1f, 0.0f};  // |Δ| = 0.1
    clear_rt.notifyAction(smooth_first, 2, 0.0);
    clear_rt.notifyAction(smooth_jump, 2, 0.01);
    clear_rt.notifyAction(smooth_next, 2, 0.02);
    const TickOutput clear_out =
        clear_rt.tick(0.03, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(clear_out.safety == SafetyLevel::kOk);
    CHECK(!clear_out.latched);
  }

  // [8] Snapshot flags are merged into the notifyAction state (AND for finite,
  //     OR for jump), not overwritten by it.
  {
    Runtime snapshot_rt(RuntimeConfig{});
    RobotSnapshot not_finite;
    not_finite.action_finite = false;
    CHECK(snapshot_rt.tick(0.0, not_finite, ActionSet{}, MotionCommand{}, true,
                           KeyPulse{})
              .safety == SafetyLevel::kCritical);

    Runtime snapshot_jump_rt(RuntimeConfig{});
    RobotSnapshot jumped;
    jumped.action_jump = true;
    CHECK(snapshot_jump_rt.tick(0.0, jumped, ActionSet{}, MotionCommand{}, true,
                                KeyPulse{})
              .safety == SafetyLevel::kCritical);
  }

  // [9] allowTransition: PASSIVE is always allowed, DEGRADED blocks nothing,
  //     and an unlatchable-critical (latch) blocks every other target.
  {
    Runtime degraded_rt(RuntimeConfig{});
    degraded_rt.tick(0.00, good, ActionSet{}, MotionCommand{}, true, tap("3"));
    RobotSnapshot degraded_robot;
    degraded_robot.pitch = 0.4f;
    const TickOutput degraded = degraded_rt.tick(0.02, degraded_robot, ActionSet{},
                                                MotionCommand{}, true, KeyPulse{});
    CHECK(degraded.safety == SafetyLevel::kDegraded);
    CHECK(!degraded.latched);
    CHECK(degraded_rt.allowTransition(Behavior::kStand));
    CHECK(degraded_rt.allowTransition(Behavior::kVelocity));
    CHECK(degraded_rt.allowTransition(Behavior::kPassive));

    Runtime open_rt(RuntimeConfig{});
    CHECK(open_rt.allowTransition(Behavior::kStand));
    CHECK(open_rt.allowTransition(Behavior::kLie));
    CHECK(open_rt.allowTransition(Behavior::kVelocity));
    CHECK(open_rt.allowTransition(Behavior::kPassive));

    // While latched every non-passive target is blocked, not just kStand.
    Runtime latched_rt(RuntimeConfig{});
    const TickOutput latched_out =
        latched_rt.tick(0.0, good, ActionSet{}, MotionCommand{}, true, tap("x"));
    CHECK(latched_out.latched);
    CHECK(!latched_rt.allowTransition(Behavior::kStand));
    CHECK(!latched_rt.allowTransition(Behavior::kLie));
    CHECK(!latched_rt.allowTransition(Behavior::kVelocity));
    CHECK(latched_rt.allowTransition(Behavior::kPassive));
  }

  // [10] setCurrentBehavior calibrates the router's fall-through, and command()
  //      mirrors the mux output (not the pre-slew target).
  {
    Runtime calib_rt(RuntimeConfig{});
    CHECK(calib_rt.currentBehavior() == Behavior::kPassive);
    calib_rt.setCurrentBehavior(Behavior::kVelocity);
    CHECK(calib_rt.currentBehavior() == Behavior::kVelocity);

    const TickOutput held =
        calib_rt.tick(0.00, good, ActionSet{}, MotionCommand{}, true, KeyPulse{});
    CHECK(held.behavior == Behavior::kVelocity);

    const TickOutput step =
        calib_rt.tick(0.02, good, ActionSet{}, MotionCommand{}, true, tap("w"));
    CHECK(std::fabs(step.command.vx - 0.04f) < 1e-6f);
    CHECK(std::fabs(calib_rt.command().vx - step.command.vx) < 1e-6f);
    CHECK(std::fabs(calib_rt.command().vx - 0.1f) > 1e-6f);  // not the target

    // The very first tick after construction integrates dt = 0 (there is no
    // previous sample to look back to), so a runtime already calibrated to
    // kVelocity emits no motion before its second tick.
    Runtime first_rt(RuntimeConfig{});
    first_rt.setCurrentBehavior(Behavior::kVelocity);
    const TickOutput first_tick =
        first_rt.tick(5.0, good, ActionSet{}, MotionCommand{}, true, tap("w"));
    CHECK(first_tick.behavior == Behavior::kVelocity);
    CHECK(first_tick.command.vx == 0.0f);
  }
}
