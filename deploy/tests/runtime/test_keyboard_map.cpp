// deploy/tests/runtime/test_keyboard_map.cpp
#include <cmath>

#include "runtime/keyboard_map.h"
#include "test_framework.h"

using namespace runtime;

namespace {

KeyPulse tap(const char* key) { return KeyPulse{key, true}; }

bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 1e-6f;
}

// True only when exactly the flag owned by `which` is set and the other five
// flags are clear.
bool only(const ActionSet& a, char which) {
  return a.stand == (which == '1') && a.lie == (which == '2') &&
         a.velocity == (which == '3') && a.passive == (which == '0') &&
         a.estop == (which == 'x') && a.rearm == (which == 'r');
}

bool none(const ActionSet& a) {
  return !a.stand && !a.lie && !a.velocity && !a.passive && !a.estop &&
         !a.rearm;
}

void check_action_keys() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});

  CHECK(only(km.onKey(tap("1")), '1'));  // stand
  CHECK(only(km.onKey(tap("2")), '2'));  // lie
  CHECK(only(km.onKey(tap("0")), '0'));  // passive
  CHECK(only(km.onKey(tap("3")), '3'));  // velocity
  CHECK(only(km.onKey(tap("x")), 'x'));  // estop
  CHECK(only(km.onKey(tap("r")), 'r'));  // rearm
}

void check_slow_mode() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});

  CHECK(km.setpoint().vx == 0.0f);
  CHECK(!km.slowMode());

  km.onKey(tap("w"));
  CHECK(near(km.setpoint().vx, 0.1f));
  for (int i = 0; i < 20; ++i) km.onKey(tap("w"));
  CHECK(near(km.setpoint().vx, 1.0f));  // 21 taps clamp at max_vx.

  // Slow mode scales each step by slow_scale (0.4), so one "s" is 0.04.
  km.onKey(tap("z"));
  CHECK(km.slowMode());
  km.onKey(tap("s"));
  CHECK(near(km.setpoint().vx, 1.0f - 0.04f));  // 0.96

  // Toggling slow mode off restores the full 0.1 step.
  km.onKey(tap("z"));
  CHECK(!km.slowMode());
  km.onKey(tap("s"));
  CHECK(near(km.setpoint().vx, 0.96f - 0.1f));  // 0.86, not another 0.04

  // Slow steps still clamp to the configured limit.
  km.onKey(tap("z"));
  CHECK(km.slowMode());
  for (int i = 0; i < 40; ++i) km.onKey(tap("s"));
  CHECK(km.setpoint().vx == -0.5f);
}

void check_vy_and_wz() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});

  km.onKey(tap("a"));
  CHECK(near(km.setpoint().vy, 0.1f));
  km.onKey(tap("d"));
  CHECK(near(km.setpoint().vy, 0.0f));

  km.onKey(tap("q"));
  CHECK(near(km.setpoint().wz, 0.2f));
  km.onKey(tap(" "));
  CHECK(km.setpoint().wz == 0.0f);
  km.onKey(tap("e"));
  CHECK(near(km.setpoint().wz, -0.2f));

  KeyboardMap vy_hi(KeyboardConfig{}, CommandLimits{});
  for (int i = 0; i < 10; ++i) vy_hi.onKey(tap("a"));
  CHECK(vy_hi.setpoint().vy == 0.5f);

  KeyboardMap vy_lo(KeyboardConfig{}, CommandLimits{});
  for (int i = 0; i < 10; ++i) vy_lo.onKey(tap("d"));
  CHECK(vy_lo.setpoint().vy == -0.5f);

  KeyboardMap wz_hi(KeyboardConfig{}, CommandLimits{});
  for (int i = 0; i < 6; ++i) wz_hi.onKey(tap("q"));
  CHECK(wz_hi.setpoint().wz == 1.0f);

  KeyboardMap wz_lo(KeyboardConfig{}, CommandLimits{});
  for (int i = 0; i < 6; ++i) wz_lo.onKey(tap("e"));
  CHECK(wz_lo.setpoint().wz == -1.0f);
}

void check_released_pulses() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});
  km.onKey(tap("w"));
  km.onKey(tap("w"));
  CHECK(near(km.setpoint().vx, 0.2f));

  // A released pulse must not adjust the setpoint.
  const float vx_before = km.setpoint().vx;
  CHECK(none(km.onKey(KeyPulse{"w", false})));
  CHECK(km.setpoint().vx == vx_before);

  // A released pulse must not toggle slow mode.
  const bool slow_before = km.slowMode();
  CHECK(!slow_before);
  km.onKey(KeyPulse{"z", false});
  CHECK(km.slowMode() == slow_before);

  // A released action key must not raise its flag.
  CHECK(none(km.onKey(KeyPulse{"x", false})));
}

void check_unknown_key() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});
  km.onKey(tap("w"));
  km.onKey(tap("a"));
  km.onKey(tap("q"));

  // Use a nonzero setpoint so "unchanged" is meaningful.
  const MotionCommand before = km.setpoint();
  CHECK(before.vx != 0.0f);
  CHECK(before.vy != 0.0f);
  CHECK(before.wz != 0.0f);

  CHECK(none(km.onKey(tap("?"))));
  CHECK(km.setpoint().vx == before.vx);
  CHECK(km.setpoint().vy == before.vy);
  CHECK(km.setpoint().wz == before.wz);
}

void check_space_stop() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});
  km.onKey(tap("w"));
  km.onKey(tap("a"));
  km.onKey(tap("q"));
  km.onKey(tap("z"));  // slow on
  CHECK(km.slowMode());
  CHECK(km.setpoint().vx != 0.0f);
  CHECK(km.setpoint().vy != 0.0f);
  CHECK(km.setpoint().wz != 0.0f);

  // Space zeroes the setpoint but does not toggle slow mode.
  km.onKey(tap(" "));
  CHECK(km.setpoint().vx == 0.0f);
  CHECK(km.setpoint().vy == 0.0f);
  CHECK(km.setpoint().wz == 0.0f);
  CHECK(km.slowMode());
}

}  // namespace

void test_keyboard_map() {
  check_action_keys();
  check_slow_mode();
  check_vy_and_wz();
  check_released_pulses();
  check_unknown_key();
  check_space_stop();
}
