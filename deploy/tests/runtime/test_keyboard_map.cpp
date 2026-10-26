// deploy/tests/runtime/test_keyboard_map.cpp
#include <cmath>

#include "runtime/keyboard_map.h"
#include "test_framework.h"

using namespace runtime;

namespace {
KeyPulse tap(const char* key) { return KeyPulse{key, true}; }
}  // namespace

void test_keyboard_map() {
  KeyboardMap km(KeyboardConfig{}, CommandLimits{});

  CHECK(km.onKey(tap("1")).stand);
  CHECK(km.onKey(tap("2")).lie);
  CHECK(km.onKey(tap("0")).passive);
  CHECK(km.onKey(tap("3")).velocity);
  CHECK(km.onKey(tap("x")).estop);
  CHECK(km.onKey(tap("r")).rearm);
  const ActionSet unknown = km.onKey(tap("?"));
  CHECK(!unknown.stand && !unknown.lie && !unknown.estop);

  CHECK(km.setpoint().vx == 0.0f);
  km.onKey(tap("w"));
  CHECK(std::fabs(km.setpoint().vx - 0.1f) < 1e-6f);
  for (int i = 0; i < 20; ++i) km.onKey(tap("w"));
  CHECK(std::fabs(km.setpoint().vx - 1.0f) < 1e-6f);

  km.onKey(tap("z"));
  for (int i = 0; i < 40; ++i) km.onKey(tap("s"));
  CHECK(std::fabs(km.setpoint().vx + 0.5f) < 1e-6f);

  km.onKey(tap("q"));
  CHECK(km.setpoint().wz > 0.0f);

  km.onKey(tap(" "));
  CHECK(km.setpoint().vx == 0.0f);
  CHECK(km.setpoint().vy == 0.0f);
  CHECK(km.setpoint().wz == 0.0f);

  const ActionSet released = km.onKey(KeyPulse{"w", false});
  CHECK(!released.stand && !released.estop);
}
