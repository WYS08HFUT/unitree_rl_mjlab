// Demo harness: drives the real runtime::Runtime with a scripted input
// timeline and prints the resulting behavior/latch state at 50 Hz.
#include <cstdio>

#include "runtime/runtime.h"

int main() {
  runtime::RuntimeConfig cfg;
  cfg.velocity_enabled = false;
  runtime::Runtime rt(cfg);

  struct Event {
    double t;
    const char* key;
  };
  const Event events[] = {
      {0.0, "1"},  // stand
      {2.5, "2"},  // lie
      {4.5, "1"},  // stand
      {7.5, "x"},  // software E-stop -> passive + latch
  };
  constexpr int kEventCount = sizeof(events) / sizeof(events[0]);
  int next = 0;

  constexpr double kDt = 0.001;
  constexpr double kEnd = 9.0;
  int tick = 0;
  for (double t = 0.0; t <= kEnd; t += kDt, ++tick) {
    runtime::KeyPulse key;
    if (next < kEventCount && t >= events[next].t) {
      key.key = events[next].key;
      key.just_pressed = true;
      ++next;
    }
    runtime::RobotSnapshot snapshot;
    const runtime::TickOutput out =
        rt.tick(t, snapshot, runtime::ActionSet{}, runtime::MotionCommand{},
                false, key);
    if (tick % 20 == 0) {
      std::printf("%.3f,%d,%d,%d\n", t, static_cast<int>(out.behavior),
                  out.latched ? 1 : 0, static_cast<int>(out.safety));
    }
  }
  return 0;
}
