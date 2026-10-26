// deploy/tests/runtime/test_command_mux.cpp
#include <cmath>

#include "runtime/command_mux.h"
#include "test_framework.h"

using namespace runtime;

void test_command_mux() {
  const CommandMuxConfig cfg;
  const CommandLimits limits;
  CommandMux mux(cfg, limits);

  MotionCommand out = mux.update(MotionCommand{5.0f, -5.0f, 9.0f}, true,
                                 SafetyLevel::kOk, 1.0);
  CHECK(std::fabs(out.vx - 1.0f) < 1e-6f);
  CHECK(std::fabs(out.vy + 0.5f) < 1e-6f);
  CHECK(std::fabs(out.wz - 1.0f) < 1e-6f);

  out = mux.update(MotionCommand{1.0f, 0.0f, 0.0f}, false, SafetyLevel::kOk, 0.1);
  CHECK(out.vx == 0.0f);

  out = mux.update(MotionCommand{0.5f, 0.0f, 0.0f}, true,
                   SafetyLevel::kDegraded, 0.1);
  CHECK(out.vx == 0.0f);

  CommandMux slew(cfg, limits);
  out = slew.update(MotionCommand{1.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk, 0.1);
  CHECK(std::fabs(out.vx - 0.2f) < 1e-6f);
  for (int i = 0; i < 10; ++i) {
    out = slew.update(MotionCommand{1.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk, 0.1);
  }
  CHECK(std::fabs(out.vx - 1.0f) < 1e-6f);

  CommandMux deadzone(cfg, limits);
  out = deadzone.update(MotionCommand{0.01f, 0.0f, 0.0f}, true,
                        SafetyLevel::kOk, 0.1);
  CHECK(out.vx == 0.0f);

  // Only kOk is safe: kCritical must zero exactly like kDegraded. An
  // implementation that special-cases kDegraded alone would leak vx here.
  out = mux.update(MotionCommand{0.5f, 0.0f, 0.0f}, true,
                   SafetyLevel::kCritical, 0.1);
  CHECK(out.vx == 0.0f);

  // Asymmetric clamp on the negative side of vx: min_vx is -0.5, not -1.0.
  CommandMux neg(cfg, limits);
  out = neg.update(MotionCommand{-5.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk, 1.0);
  CHECK(std::fabs(out.vx + 0.5f) < 1e-6f);

  // target() is the clamped goal; output() is the slew-limited ramp. An
  // implementation that stored the raw 5.0 as target would fail the first.
  CommandMux state(cfg, limits);
  state.update(MotionCommand{5.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk, 0.1);
  CHECK(std::fabs(state.target().vx - 1.0f) < 1e-6f);
  CHECK(std::fabs(state.output().vx - 0.2f) < 1e-6f);

  // wz ramps at yaw_accel_limit (4.0 * 0.1 = 0.4), not accel_limit (0.2).
  CommandMux yaw(cfg, limits);
  out = yaw.update(MotionCommand{0.0f, 0.0f, 5.0f}, true, SafetyLevel::kOk, 0.1);
  CHECK(std::fabs(out.wz - 0.4f) < 1e-6f);
  // vy ramps at accel_limit and clamps at min_vy: 0 -> -0.2, not -0.5.
  out = yaw.update(MotionCommand{0.0f, -5.0f, 0.0f}, true, SafetyLevel::kOk, 0.1);
  CHECK(std::fabs(out.vy + 0.2f) < 1e-6f);

  // Deadzone is symmetric: negative small input is zeroed too.
  out = deadzone.update(MotionCommand{-0.01f, 0.0f, 0.0f}, true,
                        SafetyLevel::kOk, 0.1);
  CHECK(out.vx == 0.0f);
  // Just above the 0.05 threshold passes through unmodified (dt=1.0 keeps
  // slew limiting out of the way).
  out = deadzone.update(MotionCommand{0.06f, 0.0f, 0.0f}, true,
                        SafetyLevel::kOk, 1.0);
  CHECK(std::fabs(out.vx - 0.06f) < 1e-6f);

  // Inactive resets internal state, not just the returned copy: the next
  // active tick must ramp from 0 (0.2), not resume from the stale 1.0.
  CommandMux reset(cfg, limits);
  reset.update(MotionCommand{1.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk, 1.0);
  out = reset.update(MotionCommand{1.0f, 0.0f, 0.0f}, false, SafetyLevel::kOk,
                     0.1);
  CHECK(out.vx == 0.0f);
  CHECK(reset.target().vx == 0.0f);
  out = reset.update(MotionCommand{1.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk,
                     0.1);
  CHECK(std::fabs(out.vx - 0.2f) < 1e-6f);

  // Degraded likewise clears both target() and output() before re-arming.
  CommandMux reset2(cfg, limits);
  reset2.update(MotionCommand{1.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk, 1.0);
  out = reset2.update(MotionCommand{1.0f, 0.0f, 0.0f}, true,
                      SafetyLevel::kDegraded, 0.1);
  CHECK(out.vx == 0.0f);
  CHECK(reset2.output().vx == 0.0f);
  out = reset2.update(MotionCommand{1.0f, 0.0f, 0.0f}, true, SafetyLevel::kOk,
                      0.1);
  CHECK(std::fabs(out.vx - 0.2f) < 1e-6f);
}
