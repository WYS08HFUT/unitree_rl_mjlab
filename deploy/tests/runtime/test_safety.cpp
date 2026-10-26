// deploy/tests/runtime/test_safety.cpp
#include <limits>

#include "runtime/safety.h"
#include "test_framework.h"

using namespace runtime;

void test_safety() {
  const SafetyConfig cfg;
  SafetySupervisor supervisor(cfg);
  RobotSnapshot good;

  CHECK(supervisor.update(good, false, 0.0) == SafetyLevel::kOk);

  RobotSnapshot tilt;
  tilt.roll = 0.4f;
  CHECK(supervisor.update(tilt, false, 0.1) == SafetyLevel::kDegraded);
  CHECK(supervisor.update(good, false, 0.2) == SafetyLevel::kDegraded);
  CHECK(supervisor.update(good, false, 0.6) == SafetyLevel::kDegraded);
  CHECK(supervisor.update(good, false, 0.75) == SafetyLevel::kOk);

  RobotSnapshot hard;
  hard.pitch = 0.7f;
  CHECK(supervisor.update(hard, false, 0.8) == SafetyLevel::kCritical);
  CHECK(supervisor.latched());
  CHECK(supervisor.update(good, false, 0.9) == SafetyLevel::kCritical);
  CHECK(!supervisor.tryRearm(1.0));
  CHECK(!supervisor.tryRearm(1.35));
  CHECK(supervisor.tryRearm(1.45));
  CHECK(!supervisor.latched());
  CHECK(supervisor.update(good, false, 1.5) == SafetyLevel::kOk);

  SafetySupervisor nan_guard(cfg);
  RobotSnapshot nan;
  nan.action_finite = false;
  CHECK(nan_guard.update(nan, false, 0.0) == SafetyLevel::kCritical);
  CHECK(nan_guard.latched());

  SafetySupervisor joint_guard(cfg);
  RobotSnapshot fast;
  fast.max_joint_vel = 35.0f;
  CHECK(joint_guard.update(fast, false, 0.0) == SafetyLevel::kDegraded);
  RobotSnapshot too_fast;
  too_fast.max_joint_vel = 55.0f;
  CHECK(joint_guard.update(too_fast, false, 0.01) == SafetyLevel::kCritical);

  SafetySupervisor estop_guard(cfg);
  CHECK(estop_guard.update(good, true, 0.0) == SafetyLevel::kCritical);
  CHECK(estop_guard.latched());

  // ---- Added assertions: thresholds, recovery, latch, rearm ----

  // Initial state and the level() accessor.
  SafetySupervisor fresh(cfg);
  CHECK(fresh.level() == SafetyLevel::kOk);
  CHECK(!fresh.latched());

  // Thresholds are strict `>`: a value exactly at a config threshold is not
  // yet a violation at that level.
  SafetySupervisor at_edge(cfg);
  RobotSnapshot edge_degraded;
  edge_degraded.roll = cfg.tilt_degraded;  // 0.35 exactly: still kOk
  CHECK(at_edge.update(edge_degraded, false, 0.0) == SafetyLevel::kOk);
  RobotSnapshot edge_critical;
  edge_critical.pitch = -cfg.tilt_critical;  // |pitch| == 0.60 exactly: kDegraded only
  CHECK(at_edge.update(edge_critical, false, 0.01) == SafetyLevel::kDegraded);

  SafetySupervisor joint_edge(cfg);
  RobotSnapshot joint_degraded_edge;
  joint_degraded_edge.max_joint_vel = cfg.joint_vel_degraded;  // 30 exactly: kOk
  CHECK(joint_edge.update(joint_degraded_edge, false, 0.0) == SafetyLevel::kOk);
  RobotSnapshot joint_critical_edge;
  joint_critical_edge.max_joint_vel = cfg.joint_vel_critical;  // 50 exactly: kDegraded
  CHECK(joint_edge.update(joint_critical_edge, false, 0.01) == SafetyLevel::kDegraded);

  // Gyro thresholds (the brief never exercised gyro_norm).
  SafetySupervisor gyro_guard(cfg);
  RobotSnapshot gyro_fast;
  gyro_fast.gyro_norm = 4.0f;
  CHECK(gyro_guard.update(gyro_fast, false, 0.0) == SafetyLevel::kDegraded);
  RobotSnapshot gyro_wild;
  gyro_wild.gyro_norm = 7.0f;
  CHECK(gyro_guard.update(gyro_wild, false, 0.01) == SafetyLevel::kCritical);
  CHECK(gyro_guard.latched());

  SafetySupervisor gyro_edge(cfg);
  RobotSnapshot gyro_degraded_edge;
  gyro_degraded_edge.gyro_norm = cfg.gyro_degraded;  // 3.5 exactly: kOk
  CHECK(gyro_edge.update(gyro_degraded_edge, false, 0.0) == SafetyLevel::kOk);
  RobotSnapshot gyro_critical_edge;
  gyro_critical_edge.gyro_norm = cfg.gyro_critical;  // 6.0 exactly: kDegraded
  CHECK(gyro_edge.update(gyro_critical_edge, false, 0.01) == SafetyLevel::kDegraded);

  // An action jump is critical and latches (the brief only covered non-finite).
  SafetySupervisor jump_guard(cfg);
  RobotSnapshot jump;
  jump.action_jump = true;
  CHECK(jump_guard.update(jump, false, 0.0) == SafetyLevel::kCritical);
  CHECK(jump_guard.latched());

  // Tilt uses |roll| and |pitch|: negative angles must trip it too.
  SafetySupervisor negative_guard(cfg);
  RobotSnapshot negative_tilt;
  negative_tilt.pitch = -0.7f;
  CHECK(negative_guard.update(negative_tilt, false, 0.0) == SafetyLevel::kCritical);

  // The CRITICAL latch survives a long run of OK ticks.
  SafetySupervisor latch_guard(cfg);
  RobotSnapshot critical_pose;
  critical_pose.roll = 1.0f;
  CHECK(latch_guard.update(critical_pose, false, 0.0) == SafetyLevel::kCritical);
  for (int i = 1; i <= 5; ++i) {
    CHECK(latch_guard.update(good, false, 0.1 * i) == SafetyLevel::kCritical);
  }
  CHECK(latch_guard.latched());
  CHECK(latch_guard.level() == SafetyLevel::kCritical);

  // DEGRADED never latches, and level() reflects it.
  SafetySupervisor soft_guard(cfg);
  RobotSnapshot soft_tilt;
  soft_tilt.roll = 0.4f;
  CHECK(soft_guard.update(soft_tilt, false, 0.0) == SafetyLevel::kDegraded);
  CHECK(!soft_guard.latched());
  CHECK(soft_guard.level() == SafetyLevel::kDegraded);

  // A degraded state escalates into a latched critical state.
  SafetySupervisor escalate(cfg);
  CHECK(escalate.update(soft_tilt, false, 0.0) == SafetyLevel::kDegraded);
  CHECK(escalate.update(too_fast, false, 0.01) == SafetyLevel::kCritical);
  CHECK(escalate.latched());

  // Re-arm needs a continuous OK streak: with none started it fails even much later.
  SafetySupervisor no_ok_guard(cfg);
  CHECK(no_ok_guard.update(critical_pose, false, 0.0) == SafetyLevel::kCritical);
  CHECK(!no_ok_guard.tryRearm(1000.0));
  CHECK(no_ok_guard.latched());

  // The streak starts at the first OK tick; later OK ticks must not restart it.
  SafetySupervisor streak_guard(cfg);
  CHECK(streak_guard.update(good, true, 0.0) == SafetyLevel::kCritical);
  CHECK(streak_guard.update(good, false, 0.25) == SafetyLevel::kCritical);
  CHECK(streak_guard.update(good, false, 0.50) == SafetyLevel::kCritical);
  CHECK(!streak_guard.tryRearm(0.74));  // 0.49 < rearm_hold_time
  CHECK(streak_guard.update(good, false, 0.75) == SafetyLevel::kCritical);
  CHECK(streak_guard.tryRearm(0.75));  // exactly 0.75 - 0.25 == rearm_hold_time
  CHECK(!streak_guard.latched());
  CHECK(streak_guard.level() == SafetyLevel::kOk);

  // A degraded tick breaks the OK streak, so the hold restarts afterwards.
  SafetySupervisor broken_streak(cfg);
  CHECK(broken_streak.update(good, true, 0.0) == SafetyLevel::kCritical);
  CHECK(broken_streak.update(good, false, 0.25) == SafetyLevel::kCritical);
  CHECK(broken_streak.update(soft_tilt, false, 0.5) == SafetyLevel::kCritical);
  CHECK(!broken_streak.tryRearm(1.0));
  CHECK(broken_streak.update(good, false, 1.25) == SafetyLevel::kCritical);
  CHECK(!broken_streak.tryRearm(1.5));  // only 0.25 of fresh OK
  CHECK(broken_streak.tryRearm(1.75));  // 0.5 of fresh OK
  CHECK(!broken_streak.latched());

  // After re-arming, a new critical event must latch again from scratch.
  SafetySupervisor relatch(cfg);
  CHECK(relatch.update(good, true, 0.0) == SafetyLevel::kCritical);
  CHECK(relatch.update(good, false, 0.0) == SafetyLevel::kCritical);
  CHECK(relatch.tryRearm(0.5));  // exactly 0.5 - 0.0 == rearm_hold_time
  CHECK(!relatch.latched());
  CHECK(relatch.update(good, true, 0.75) == SafetyLevel::kCritical);
  CHECK(relatch.latched());
  CHECK(!relatch.tryRearm(1000.0));

  // Recovery completes exactly at recover_time, not before.
  SafetySupervisor recover_guard(cfg);
  CHECK(recover_guard.update(soft_tilt, false, 0.0) == SafetyLevel::kDegraded);
  CHECK(recover_guard.update(good, false, 1.0) == SafetyLevel::kDegraded);
  CHECK(recover_guard.update(good, false, 1.25) == SafetyLevel::kDegraded);
  CHECK(recover_guard.update(good, false, 1.5) == SafetyLevel::kOk);  // 0.5 exactly
  CHECK(recover_guard.update(soft_tilt, false, 1.6) == SafetyLevel::kDegraded);

  // A degraded tick during recovery restarts the recover_time window.
  SafetySupervisor recover_reset(cfg);
  CHECK(recover_reset.update(soft_tilt, false, 0.0) == SafetyLevel::kDegraded);
  CHECK(recover_reset.update(good, false, 1.0) == SafetyLevel::kDegraded);
  CHECK(recover_reset.update(soft_tilt, false, 1.25) == SafetyLevel::kDegraded);
  CHECK(recover_reset.update(good, false, 1.5) == SafetyLevel::kDegraded);
  CHECK(recover_reset.update(good, false, 2.0) == SafetyLevel::kOk);  // 0.5 after restart

  // Non-finite robot data must fail closed: a NaN/inf snapshot value compares
  // false against every `>` threshold and would otherwise read as healthy.
  // Latching means each case needs a fresh supervisor.
  SafetySupervisor nan_roll_guard(cfg);
  RobotSnapshot nan_roll;
  nan_roll.roll = std::numeric_limits<float>::quiet_NaN();
  CHECK(nan_roll_guard.update(nan_roll, false, 0.0) == SafetyLevel::kCritical);
  CHECK(nan_roll_guard.latched());

  SafetySupervisor inf_pitch_guard(cfg);
  RobotSnapshot inf_pitch;
  inf_pitch.pitch = std::numeric_limits<float>::infinity();
  CHECK(inf_pitch_guard.update(inf_pitch, false, 0.0) == SafetyLevel::kCritical);
  CHECK(inf_pitch_guard.latched());

  SafetySupervisor nan_gyro_guard(cfg);
  RobotSnapshot nan_gyro;
  nan_gyro.gyro_norm = std::numeric_limits<float>::quiet_NaN();
  CHECK(nan_gyro_guard.update(nan_gyro, false, 0.0) == SafetyLevel::kCritical);
  CHECK(nan_gyro_guard.latched());

  SafetySupervisor nan_joint_guard(cfg);
  RobotSnapshot nan_joint;
  nan_joint.max_joint_vel = std::numeric_limits<float>::quiet_NaN();
  CHECK(nan_joint_guard.update(nan_joint, false, 0.0) == SafetyLevel::kCritical);
  CHECK(nan_joint_guard.latched());
}
