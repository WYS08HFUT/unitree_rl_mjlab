# Go2 运行时骨架 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development (recommended) or executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不训练策略的前提下，为 Go2 部署链路加入可单测的输入抽象、命令仲裁、安全监督与脚本化姿态切换。

**Architecture:** `deploy/include/runtime/` 放纯 C++17、无 SDK 依赖的核心；`deploy/robots/go2/` 的适配层负责读取 `LowState`/手柄 DSL 并驱动 FSMyi。共享 `FSMState`/`CtrlFSM` 只加默认惰性的钩子，其他机器人行为不变。

**Tech Stack:** C++17、yaml-cpp、unitree_sdk2（本地前缀 `~/unitree_sdk2/install`）、CMake + ctest。核心测试不链接任何 SDK。

**Spec:** [docs/superpowers/specs/2026-10-26-go2-runtime-skeleton-design.md](../specs/2026-10-26-go2-runtime-skeleton-design.md)

## Global Constraints

- C++17；核心头文件只依赖标准库，禁止 include `unitree/`、`yaml-cpp`、Eigen。
- 不修改其他机器人（g1/g1_23dof/h1_2/a2/as2/r1/h2）的配置；共享类新增成员默认不生效。
- 本轮 `velocity_enabled=false`，不启用 `Velocity` 状态，不加载 ONNX。
- 安全阈值、键位、表达式全部可配；默认值以 spec 为准。
- 适配层单文件 ≤ ~150 行；每任务结束提交一次。
- 本地 SDK 前缀固定为 `$HOME/unitree_sdk2/install`；构建命令必须带 include/link flags（见 Task 7/8/9）。

---

### Task 1: 测试脚手架与运行时类型

**Files:**
- Create: `deploy/tests/CMakeLists.txt`
- Create: `deploy/tests/runtime/test_framework.h`
- Create: `deploy/tests/runtime/test_main.cpp`
- Create: `deploy/tests/runtime/test_keyboard_map.cpp`
- Create: `deploy/tests/runtime/test_command_mux.cpp`
- Create: `deploy/tests/runtime/test_safety.cpp`
- Create: `deploy/tests/runtime/test_behavior.cpp`
- Create: `deploy/tests/runtime/test_replay.cpp`
- Create: `deploy/include/runtime/types.h`
- Modify: `.gitignore`（忽略 `build/`）

**Interfaces:**
- Produces: `runtime::Behavior`、`SafetyLevel`、`MotionCommand`、`CommandLimits`、`RobotSnapshot`、`ActionSet`、`KeyPulse`、`KeyboardConfig`、`CommandMuxConfig`、`SafetyConfig`、`TickOutput`、`RuntimeConfig`（字段见下）

- [ ] **Step 1: 写 `types.h`**

```cpp
// deploy/include/runtime/types.h
#pragma once

#include <string>

namespace runtime {

enum class Behavior { kPassive, kStand, kLie, kVelocity };
enum class SafetyLevel { kOk, kDegraded, kCritical };

struct MotionCommand {
  float vx{0.0f};
  float vy{0.0f};
  float wz{0.0f};
};

struct CommandLimits {
  float min_vx{-0.5f};
  float max_vx{1.0f};
  float min_vy{-0.5f};
  float max_vy{0.5f};
  float min_wz{-1.0f};
  float max_wz{1.0f};
};

struct RobotSnapshot {
  float roll{0.0f};
  float pitch{0.0f};
  float gyro_norm{0.0f};
  float max_joint_vel{0.0f};
  bool action_finite{true};
  bool action_jump{false};
};

struct ActionSet {
  bool stand{false};
  bool lie{false};
  bool velocity{false};
  bool passive{false};
  bool estop{false};
  bool rearm{false};
};

struct KeyPulse {
  std::string key;
  bool just_pressed{false};
};

struct KeyboardConfig {
  std::string stand{"1"};
  std::string lie{"2"};
  std::string velocity{"3"};
  std::string passive{"0"};
  std::string estop{"x"};
  std::string rearm{"r"};
  std::string vx_up{"w"};
  std::string vx_down{"s"};
  std::string vy_up{"a"};
  std::string vy_down{"d"};
  std::string wz_up{"q"};
  std::string wz_down{"e"};
  std::string stop{" "};
  std::string slow_toggle{"z"};
  float vx_step{0.1f};
  float vy_step{0.1f};
  float wz_step{0.2f};
  float slow_scale{0.4f};
};

struct CommandMuxConfig {
  float deadzone{0.05f};
  float accel_limit{2.0f};
  float yaw_accel_limit{4.0f};
};

struct SafetyConfig {
  float tilt_degraded{0.35f};
  float tilt_critical{0.60f};
  float gyro_degraded{3.5f};
  float gyro_critical{6.0f};
  float joint_vel_degraded{30.0f};
  float joint_vel_critical{50.0f};
  float action_jump_critical{1.5f};
  float recover_time{0.5f};
  float rearm_hold_time{0.5f};
};

struct TickOutput {
  Behavior behavior{Behavior::kPassive};
  MotionCommand command{};
  SafetyLevel safety{SafetyLevel::kOk};
  bool latched{false};
};

struct RuntimeConfig {
  bool velocity_enabled{true};
  CommandLimits limits{};
  KeyboardConfig keyboard{};
  CommandMuxConfig command{};
  SafetyConfig safety{};
};

}  // namespace runtime
```

- [ ] **Step 2: 写测试框架与入口**

```cpp
// deploy/tests/runtime/test_framework.h
#pragma once

#include <cstdio>

namespace testfw {
inline int checks = 0;
inline int failures = 0;
}  // namespace testfw

#define CHECK(cond)                                                       \
  do {                                                                    \
    ++testfw::checks;                                                     \
    if (!(cond)) {                                                        \
      ++testfw::failures;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
    }                                                                     \
  } while (0)
```

```cpp
// deploy/tests/runtime/test_main.cpp
#include <cstdio>

#include "test_framework.h"

void test_keyboard_map();
void test_command_mux();
void test_safety();
void test_behavior();
void test_replay();

int main() {
  test_keyboard_map();
  test_command_mux();
  test_safety();
  test_behavior();
  test_replay();
  std::printf("%d checks, %d failures\n", testfw::checks, testfw::failures);
  return testfw::failures == 0 ? 0 : 1;
}
```

- [ ] **Step 3: 写 5 个空的测试函数**

```cpp
// deploy/tests/runtime/test_keyboard_map.cpp
#include "test_framework.h"
void test_keyboard_map() {}
```

```cpp
// deploy/tests/runtime/test_command_mux.cpp
#include "test_framework.h"
void test_command_mux() {}
```

```cpp
// deploy/tests/runtime/test_safety.cpp
#include "test_framework.h"
void test_safety() {}
```

```cpp
// deploy/tests/runtime/test_behavior.cpp
#include "test_framework.h"
void test_behavior() {}
```

```cpp
// deploy/tests/runtime/test_replay.cpp
#include "test_framework.h"
void test_replay() {}
```

- [ ] **Step 4: 写 CMake**

```cmake
# deploy/tests/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(go2_runtime_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(runtime_tests
  runtime/test_main.cpp
  runtime/test_keyboard_map.cpp
  runtime/test_command_mux.cpp
  runtime/test_safety.cpp
  runtime/test_behavior.cpp
  runtime/test_replay.cpp)

target_include_directories(runtime_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

target_compile_options(runtime_tests PRIVATE -Wall -Wextra)

add_test(NAME runtime_tests COMMAND runtime_tests)
```

- [ ] **Step 5: 让 `.gitignore` 忽略构建目录**

在 `.gitignore` 末尾追加：

```gitignore
build
```

- [ ] **Step 6: 构建并运行（应为 0 checks）**

```bash
cmake -S deploy/tests -B build/tests
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure
```

Expected: `0 checks, 0 failures`，ctest 通过。

- [ ] **Step 7: 提交**

```bash
git add .gitignore deploy/include/runtime/types.h deploy/tests
git commit -m "test: add runtime test scaffold and core types"
```

---

### Task 2: KeyboardMap

**Files:**
- Create: `deploy/include/runtime/keyboard_map.h`
- Modify: `deploy/tests/runtime/test_keyboard_map.cpp`

**Interfaces:**
- Consumes: `runtime::KeyboardConfig`、`CommandLimits`、`ActionSet`、`KeyPulse`（Task 1）
- Produces: `runtime::KeyboardMap(KeyboardConfig, CommandLimits)`、`ActionSet onKey(const KeyPulse&)`、`const MotionCommand& setpoint()`、`bool slowMode()`

- [ ] **Step 1: 写失败测试**

```cpp
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
```

- [ ] **Step 2: 运行确认失败**

```bash
cmake --build build/tests -j
```

Expected: 编译失败，`runtime/keyboard_map.h: No such file or directory`。

- [ ] **Step 3: 实现**

```cpp
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
```

- [ ] **Step 4: 运行确认通过**

```bash
cmake --build build/tests -j && ctest --test-dir build/tests --output-on-failure
```

Expected: 全部通过。

- [ ] **Step 5: 提交**

```bash
git add deploy/include/runtime/keyboard_map.h deploy/tests/runtime/test_keyboard_map.cpp
git commit -m "feat: add keyboard action and setpoint mapping"
```

---

### Task 3: CommandMux

**Files:**
- Create: `deploy/include/runtime/command_mux.h`
- Modify: `deploy/tests/runtime/test_command_mux.cpp`

**Interfaces:**
- Consumes: `CommandMuxConfig`、`CommandLimits`、`MotionCommand`、`SafetyLevel`
- Produces: `CommandMux(CommandMuxConfig, CommandLimits)`、`MotionCommand update(const MotionCommand&, bool active, SafetyLevel, double dt)`、`output()`、`target()`

- [ ] **Step 1: 写失败测试**

```cpp
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
}
```

- [ ] **Step 2: 运行确认失败**

```bash
cmake --build build/tests -j
```

Expected: `runtime/command_mux.h: No such file or directory`。

- [ ] **Step 3: 实现**

```cpp
// deploy/include/runtime/command_mux.h
#pragma once

#include <algorithm>
#include <cmath>

#include "runtime/types.h"

namespace runtime {

class CommandMux {
 public:
  CommandMux(CommandMuxConfig cfg, CommandLimits limits)
      : cfg_(cfg), limits_(limits) {}

  MotionCommand update(const MotionCommand& manual, bool active,
                       SafetyLevel safety, double dt) {
    if (!active || safety != SafetyLevel::kOk) {
      target_ = MotionCommand{};
      output_ = MotionCommand{};
      return output_;
    }

    target_.vx = std::clamp(deadzone(manual.vx), limits_.min_vx, limits_.max_vx);
    target_.vy = std::clamp(deadzone(manual.vy), limits_.min_vy, limits_.max_vy);
    target_.wz = std::clamp(deadzone(manual.wz), limits_.min_wz, limits_.max_wz);

    output_.vx = slew(output_.vx, target_.vx, cfg_.accel_limit * dt);
    output_.vy = slew(output_.vy, target_.vy, cfg_.accel_limit * dt);
    output_.wz = slew(output_.wz, target_.wz, cfg_.yaw_accel_limit * dt);
    return output_;
  }

  const MotionCommand& target() const { return target_; }
  const MotionCommand& output() const { return output_; }

 private:
  float deadzone(float v) const {
    return std::fabs(v) < cfg_.deadzone ? 0.0f : v;
  }

  static float slew(float current, float target, float max_delta) {
    const float delta = target - current;
    if (std::fabs(delta) <= max_delta) return target;
    return current + (delta > 0.0f ? max_delta : -max_delta);
  }

  CommandMuxConfig cfg_;
  CommandLimits limits_;
  MotionCommand target_{};
  MotionCommand output_{};
};

}  // namespace runtime
```

- [ ] **Step 4: 运行确认通过**

```bash
cmake --build build/tests -j && ctest --test-dir build/tests --output-on-failure
```

- [ ] **Step 5: 提交**

```bash
git add deploy/include/runtime/command_mux.h deploy/tests/runtime/test_command_mux.cpp
git commit -m "feat: add command mux with limits and slew"
```

---

### Task 4: SafetySupervisor

**Files:**
- Create: `deploy/include/runtime/safety.h`
- Modify: `deploy/tests/runtime/test_safety.cpp`

**Interfaces:**
- Consumes: `SafetyConfig`、`RobotSnapshot`
- Produces: `SafetySupervisor(SafetyConfig)`、`SafetyLevel update(const RobotSnapshot&, bool estop, double now)`、`bool tryRearm(double now)`、`bool latched()`、`SafetyLevel level()`

- [ ] **Step 1: 写失败测试**

```cpp
// deploy/tests/runtime/test_safety.cpp
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
}
```

- [ ] **Step 2: 运行确认失败**

```bash
cmake --build build/tests -j
```

Expected: `runtime/safety.h: No such file or directory`。

- [ ] **Step 3: 实现**

```cpp
// deploy/include/runtime/safety.h
#pragma once

#include <algorithm>
#include <cmath>

#include "runtime/types.h"

namespace runtime {

class SafetySupervisor {
 public:
  explicit SafetySupervisor(SafetyConfig cfg) : cfg_(cfg) {}

  SafetyLevel update(const RobotSnapshot& robot, bool estop, double now) {
    const SafetyLevel raw = rawLevel(robot, estop);

    if (raw == SafetyLevel::kCritical) {
      latched_ = true;
      level_ = SafetyLevel::kCritical;
      ok_since_ = kNoTime;
      return level_;
    }

    if (latched_) {
      if (raw == SafetyLevel::kOk) {
        if (ok_since_ == kNoTime) ok_since_ = now;
      } else {
        ok_since_ = kNoTime;
      }
      level_ = SafetyLevel::kCritical;
      return level_;
    }

    if (raw == SafetyLevel::kDegraded) {
      level_ = SafetyLevel::kDegraded;
      ok_since_ = kNoTime;
      return level_;
    }

    if (ok_since_ == kNoTime) ok_since_ = now;
    if (level_ == SafetyLevel::kDegraded &&
        now - ok_since_ < cfg_.recover_time) {
      return level_;
    }
    level_ = SafetyLevel::kOk;
    return level_;
  }

  bool tryRearm(double now) {
    if (!latched_ || ok_since_ == kNoTime) return false;
    if (now - ok_since_ < cfg_.rearm_hold_time) return false;
    latched_ = false;
    level_ = SafetyLevel::kOk;
    return true;
  }

  bool latched() const { return latched_; }
  SafetyLevel level() const { return level_; }

 private:
  static constexpr double kNoTime = -1e9;

  SafetyLevel rawLevel(const RobotSnapshot& robot, bool estop) const {
    if (estop || !robot.action_finite || robot.action_jump) {
      return SafetyLevel::kCritical;
    }
    const float tilt = std::max(std::fabs(robot.roll), std::fabs(robot.pitch));
    if (tilt > cfg_.tilt_critical || robot.gyro_norm > cfg_.gyro_critical ||
        robot.max_joint_vel > cfg_.joint_vel_critical) {
      return SafetyLevel::kCritical;
    }
    if (tilt > cfg_.tilt_degraded || robot.gyro_norm > cfg_.gyro_degraded ||
        robot.max_joint_vel > cfg_.joint_vel_degraded) {
      return SafetyLevel::kDegraded;
    }
    return SafetyLevel::kOk;
  }

  SafetyConfig cfg_;
  SafetyLevel level_{SafetyLevel::kOk};
  bool latched_{false};
  double ok_since_{kNoTime};
};

}  // namespace runtime
```

- [ ] **Step 4: 运行确认通过**

```bash
cmake --build build/tests -j && ctest --test-dir build/tests --output-on-failure
```

- [ ] **Step 5: 提交**

```bash
git add deploy/include/runtime/safety.h deploy/tests/runtime/test_safety.cpp
git commit -m "feat: add tiered safety supervisor with latch and rearm"
```

---

### Task 5: BehaviorRouter

**Files:**
- Create: `deploy/include/runtime/behavior.h`
- Modify: `deploy/tests/runtime/test_behavior.cpp`

**Interfaces:**
- Consumes: `ActionSet`、`SafetySupervisor`、`Behavior`
- Produces: `BehaviorRouter()`、`Behavior update(const ActionSet&, SafetySupervisor&, Behavior current, double now, bool velocity_enabled)`

- [ ] **Step 1: 写失败测试**

```cpp
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
}
```

- [ ] **Step 2: 运行确认失败**

```bash
cmake --build build/tests -j
```

Expected: `runtime/behavior.h: No such file or directory`。

- [ ] **Step 3: 实现**

```cpp
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
      if (actions.rearm && safety.tryRearm(now)) {
        return Behavior::kPassive;
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
```

- [ ] **Step 4: 运行确认通过**

```bash
cmake --build build/tests -j && ctest --test-dir build/tests --output-on-failure
```

- [ ] **Step 5: 提交**

```bash
git add deploy/include/runtime/behavior.h deploy/tests/runtime/test_behavior.cpp
git commit -m "feat: add behavior router with safety gating"
```

---

### Task 6: Runtime 编排与回放测试

**Files:**
- Create: `deploy/include/runtime/runtime.h`
- Modify: `deploy/tests/runtime/test_replay.cpp`

**Interfaces:**
- Consumes: `KeyboardMap`、`CommandMux`、`SafetySupervisor`、`BehaviorRouter`、`RuntimeConfig`
- Produces: `Runtime(RuntimeConfig)`、`TickOutput tick(double now, const RobotSnapshot&, const ActionSet&, const MotionCommand&, bool gamepad_fresh, const KeyPulse&)`、`void notifyAction(const float*, std::size_t, double)`、`bool allowTransition(Behavior)`、`void setCurrentBehavior(Behavior)`、`Behavior currentBehavior()`、`const MotionCommand& command()`

- [ ] **Step 1: 写失败的回放测试**

```cpp
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
}
```

- [ ] **Step 2: 运行确认失败**

```bash
cmake --build build/tests -j
```

Expected: `runtime/runtime.h: No such file or directory`。

- [ ] **Step 3: 实现**

```cpp
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
```

- [ ] **Step 4: 运行确认通过**

```bash
cmake --build build/tests -j && ctest --test-dir build/tests --output-on-failure
```

Expected: `N checks, 0 failures`，ctest 通过。

- [ ] **Step 5: 提交**

```bash
git add deploy/include/runtime/runtime.h deploy/tests/runtime/test_replay.cpp
git commit -m "feat: add runtime orchestrator with replay coverage"
```

---

### Task 7: 共享 FSM 钩子

**Files:**
- Create: `deploy/include/runtime/ticker.h`
- Modify: `deploy/include/FSM/FSMState.h`
- Modify: `deploy/include/FSM/CtrlFSM.h`

**Interfaces:**
- Produces: `runtime::IRuntimeTicker`（`tick()`、`allowTransition(int)`）、`FSMState::runtime`、`CtrlFSM::requestState(int)`、`CtrlFSM::setTransitionGate(std::function<bool(int)>)`、`CtrlFSM::currentStateName()`

- [ ] **Step 1: 新增接口头**

```cpp
// deploy/include/runtime/ticker.h
#pragma once

namespace runtime {

class IRuntimeTicker {
 public:
  virtual ~IRuntimeTicker() = default;
  virtual void tick() = 0;
  virtual bool allowTransition(int fsm_id) const = 0;
};

}  // namespace runtime
```

- [ ] **Step 2: 在 `FSMState` 上加静态指针与 tick**

在 `deploy/include/FSM/FSMState.h` 顶部 include：

```cpp
#include "runtime/ticker.h"
```

在 `FSMState` 的 public 区加：

```cpp
  inline static std::shared_ptr<runtime::IRuntimeTicker> runtime;
```

把 `pre_run()` 改为：

```cpp
  void pre_run()
  {
    lowstate->update();
    if(keyboard) keyboard->update();
    if(runtime) runtime->tick();
  }
```

- [ ] **Step 3: 在 `CtrlFSM` 上加请求、门与状态名**

在 `deploy/include/FSM/CtrlFSM.h` 顶部加：

```cpp
#include <functional>
#include <string>
```

在 `public:` 区加：

```cpp
    void requestState(int state) { pending_state_ = state; }

    void setTransitionGate(std::function<bool(int)> gate) {
        transition_gate_ = std::move(gate);
    }

    std::string currentStateName() const {
        return currentState ? currentState->getStateString() : std::string();
    }
```

把 `run_()` 里的状态选择改为：

```cpp
        // Check if need to change state
        int nextStateMode = 0;
        if (pending_state_ != 0) {
            nextStateMode = pending_state_;
            pending_state_ = 0;
        } else {
            for(int i(0); i<currentState->registered_checks.size(); i++)
            {
                if(currentState->registered_checks[i].first())
                {
                    nextStateMode = currentState->registered_checks[i].second;
                    break;
                }
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode) &&
           (!transition_gate_ || transition_gate_(nextStateMode)))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    break;
                }
            }
        }
```

在 `private:` 区加成员：

```cpp
    int pending_state_ = 0;
    std::function<bool(int)> transition_gate_;
```

- [ ] **Step 4: 语法检查（不链接）**

```bash
P=$HOME/unitree_sdk2/install
printf '#include "FSM/FSMState.h"\n#include "FSM/CtrlFSM.h"\n' > /tmp/fsm_hooks_check.cpp
g++ -std=c++17 -fsyntax-only \
  -I deploy/include -I deploy/robots/go2/include \
  -I "$P/include" -I "$P/include/ddscxx" \
  -I /usr/include/eigen3 /tmp/fsm_hooks_check.cpp
```

Expected: 无输出、退出码 0。

- [ ] **Step 5: 提交**

```bash
git add deploy/include/runtime/ticker.h deploy/include/FSM/FSMState.h deploy/include/FSM/CtrlFSM.h
git commit -m "feat: add runtime hooks to FSM framework"
```

---

### Task 8: State_LieDown 与 Go2 配置

**Files:**
- Create: `deploy/include/FSM/State_LieDown.h`
- Modify: `deploy/robots/go2/config/config.yaml`

**Interfaces:**
- Consumes: `FSMState`、`param::config`
- Produces: FSM 状态 `LieDown`（id 4）

- [ ] **Step 1: 新增 `State_LieDown`**

```cpp
// deploy/include/FSM/State_LieDown.h
#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"

class State_LieDown : public FSMState
{
public:
    State_LieDown(int state, std::string state_string = "LieDown")
    : FSMState(state, state_string)
    {
        ts_ = param::config["FSM"]["LieDown"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["LieDown"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
    }

    void enter()
    {
        static auto kp = param::config["FSM"]["LieDown"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["LieDown"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kp.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = motor.tau() = 0;
        }

        std::vector<float> q0;
        for(int i(0); i < kp.size(); ++i) {
            q0.push_back(lowcmd->msg_.motor_cmd()[i].q());
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        auto q = linear_interpolate(t, ts_, qs_);
        for(int i(0); i < q.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }
    }

private:
    double t0_{0.0};
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

REGISTER_FSM(State_LieDown)
```

- [ ] **Step 2: 替换 Go2 `config.yaml`**

```yaml
FSM:
  _: # enabled fsms
    Passive:
      id: 1
    FixStand:
      id: 2
    LieDown:
      id: 4
  Passive:
    transitions: {}
    mode: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    kd: [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
  FixStand:
    transitions: {}
    kp: [60, 80, 80, 60, 80, 80, 60, 80, 80, 60, 80, 80]
    kd: [5, 4, 4, 5, 4, 4, 5, 4, 4, 5, 4, 4]
    ts: [0, 1, 2]
    qs: [
      [],
      [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65],
      [0., 0.8, -1.5, 0., 0.8, -1.5, 0., 0.8, -1.5, 0., 0.8, -1.5],
    ]
  LieDown:
    transitions: {}
    kp: [60, 80, 80, 60, 80, 80, 60, 80, 80, 60, 80, 80]
    kd: [5, 4, 4, 5, 4, 4, 5, 4, 4, 5, 4, 4]
    ts: [0.0, 1.5]
    qs:
      - []
      - [0.0, 1.45, -2.70, 0.0, 1.45, -2.70, 0.0, 1.45, -2.70, 0.0, 1.45, -2.70]
  Velocity:
    policy_dir: config/policy/velocity

Runtime:
  velocity_enabled: false
  velocity_params: config/policy/velocity/v0/params/deploy.yaml
  command:
    deadzone: 0.05
    accel_limit: 2.0
    yaw_accel_limit: 4.0
  safety:
    tilt_degraded: 0.35
    tilt_critical: 0.60
    gyro_degraded: 3.5
    gyro_critical: 6.0
    joint_vel_degraded: 30.0
    joint_vel_critical: 50.0
    action_jump_critical: 1.5
    recover_time: 0.5
    rearm_hold_time: 0.5
  keyboard:
    stand: "1"
    lie: "2"
    velocity: "3"
    passive: "0"
    estop: "x"
    rearm: "r"
    vx_step: 0.1
    vy_step: 0.1
    wz_step: 0.2
    slow_scale: 0.4
    slow_toggle: "z"
  gamepad:
    stand: "LT + up.on_pressed"
    lie: "LT + down.on_pressed"
    velocity: "RT + A.on_pressed"
    passive: "LT + B.on_pressed"
    estop: "RT + B.on_pressed"
    rearm: "LT + right.on_pressed"
```

- [ ] **Step 3: 构建 `go2_ctrl`**

```bash
P=$HOME/unitree_sdk2/install
cmake -S deploy/robots/go2 -B build/go2 \
  -DCMAKE_CXX_FLAGS="-I$P/include -I$P/include/ddscxx" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$P/lib -Wl,-rpath,$P/lib"
cmake --build build/go2 -j
```

Expected: 找到 `build/go2/go2_ctrl`，退出码 0。

- [ ] **Step 4: 提交**

```bash
git add deploy/include/FSM/State_LieDown.h deploy/robots/go2/config/config.yaml
git commit -m "feat: add scripted lie-down state and runtime config"
```

---

### Task 9: 手柄适配、Bridge 与 main 接线

**Files:**
- Create: `deploy/robots/go2/include/GamepadAdapter.h`
- Create: `deploy/robots/go2/include/Go2RuntimeBridge.h`
- Create: `deploy/robots/go2/src/Go2RuntimeBridge.cpp`
- Modify: `deploy/robots/go2/main.cpp`

**Interfaces:**
- Consumes: `runtime::Runtime`、`IRuntimeTicker`、`FSMState::lowstate`、`FSMState::keyboard`、`FSMStringMap`
- Produces: `Go2RuntimeBridge`（实现 `IRuntimeTicker`）、`GamepadAdapter`

- [ ] **Step 1: 写 `GamepadAdapter`**

```cpp
// deploy/robots/go2/include/GamepadAdapter.h
#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "runtime/types.h"
#include "unitree_joystick_dsl.hpp"

class GamepadAdapter {
 public:
  struct Expressions {
    std::string stand;
    std::string lie;
    std::string velocity;
    std::string passive;
    std::string estop;
    std::string rearm;
  };

  explicit GamepadAdapter(const Expressions& expressions) {
    add("stand", expressions.stand);
    add("lie", expressions.lie);
    add("velocity", expressions.velocity);
    add("passive", expressions.passive);
    add("estop", expressions.estop);
    add("rearm", expressions.rearm);
  }

  runtime::ActionSet actions(const unitree::common::UnitreeJoystick& joystick) const {
    runtime::ActionSet actions;
    actions.stand = eval("stand", joystick);
    actions.lie = eval("lie", joystick);
    actions.velocity = eval("velocity", joystick);
    actions.passive = eval("passive", joystick);
    actions.estop = eval("estop", joystick);
    actions.rearm = eval("rearm", joystick);
    return actions;
  }

 private:
  using Predicate =
      std::function<bool(const unitree::common::UnitreeJoystick&)>;

  void add(const std::string& name, const std::string& expression) {
    if (expression.empty()) return;
    unitree::common::dsl::Parser parser(expression);
    auto ast = parser.Parse();
    predicates_[name] = unitree::common::dsl::Compile(*ast);
  }

  bool eval(const std::string& name,
            const unitree::common::UnitreeJoystick& joystick) const {
    const auto it = predicates_.find(name);
    return it != predicates_.end() && it->second(joystick);
  }

  std::unordered_map<std::string, Predicate> predicates_;
};
```

- [ ] **Step 2: 写 Bridge 头与实现**

```cpp
// deploy/robots/go2/include/Go2RuntimeBridge.h
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "GamepadAdapter.h"
#include "runtime/runtime.h"
#include "runtime/ticker.h"

class Go2RuntimeBridge : public runtime::IRuntimeTicker {
 public:
  struct Callbacks {
    std::function<void(int)> request_state;
    std::function<std::string()> current_state_name;
    std::function<int(const std::string&)> state_id;
  };

  Go2RuntimeBridge(const YAML::Node& cfg, const runtime::CommandLimits& limits,
                   const GamepadAdapter::Expressions& expressions,
                   Callbacks callbacks);

  void tick() override;
  bool allowTransition(int fsm_id) const override;

  const runtime::MotionCommand& command() const { return runtime_.command(); }

 private:
  std::optional<runtime::Behavior> behaviorFromName(const std::string& name) const;
  std::optional<int> stateId(runtime::Behavior behavior) const;

  runtime::Runtime runtime_;
  runtime::CommandLimits limits_;
  GamepadAdapter adapter_;
  Callbacks callbacks_;
  std::unordered_map<std::string, runtime::Behavior> names_;
  std::unordered_map<int, runtime::Behavior> id_to_behavior_;
};
```

```cpp
// deploy/robots/go2/src/Go2RuntimeBridge.cpp
#include "Go2RuntimeBridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

#include "FSM/FSMState.h"

namespace {

double nowSeconds() {
  using Clock = std::chrono::steady_clock;
  static const Clock::time_point start = Clock::now();
  return std::chrono::duration<double>(Clock::now() - start).count();
}

float scaleAxis(float value, float min, float max) {
  return value >= 0.0f ? value * max : value * (-min);
}

float number(const YAML::Node& node, const char* key, float fallback) {
  return node && node[key] ? node[key].as<float>() : fallback;
}

bool boolean(const YAML::Node& node, const char* key, bool fallback) {
  return node && node[key] ? node[key].as<bool>() : fallback;
}

std::string text(const YAML::Node& node, const char* key,
                 const std::string& fallback) {
  return node && node[key] ? node[key].as<std::string>() : fallback;
}

runtime::RuntimeConfig makeConfig(const YAML::Node& root,
                                  const runtime::CommandLimits& limits) {
  runtime::RuntimeConfig cfg;
  cfg.velocity_enabled = boolean(root, "velocity_enabled", false);
  cfg.limits = limits;

  const YAML::Node command = root["command"];
  cfg.command.deadzone = number(command, "deadzone", cfg.command.deadzone);
  cfg.command.accel_limit =
      number(command, "accel_limit", cfg.command.accel_limit);
  cfg.command.yaw_accel_limit =
      number(command, "yaw_accel_limit", cfg.command.yaw_accel_limit);

  const YAML::Node safety = root["safety"];
  cfg.safety.tilt_degraded =
      number(safety, "tilt_degraded", cfg.safety.tilt_degraded);
  cfg.safety.tilt_critical =
      number(safety, "tilt_critical", cfg.safety.tilt_critical);
  cfg.safety.gyro_degraded =
      number(safety, "gyro_degraded", cfg.safety.gyro_degraded);
  cfg.safety.gyro_critical =
      number(safety, "gyro_critical", cfg.safety.gyro_critical);
  cfg.safety.joint_vel_degraded =
      number(safety, "joint_vel_degraded", cfg.safety.joint_vel_degraded);
  cfg.safety.joint_vel_critical =
      number(safety, "joint_vel_critical", cfg.safety.joint_vel_critical);
  cfg.safety.action_jump_critical = number(
      safety, "action_jump_critical", cfg.safety.action_jump_critical);
  cfg.safety.recover_time =
      number(safety, "recover_time", cfg.safety.recover_time);
  cfg.safety.rearm_hold_time =
      number(safety, "rearm_hold_time", cfg.safety.rearm_hold_time);

  const YAML::Node keyboard = root["keyboard"];
  cfg.keyboard.stand = text(keyboard, "stand", cfg.keyboard.stand);
  cfg.keyboard.lie = text(keyboard, "lie", cfg.keyboard.lie);
  cfg.keyboard.velocity = text(keyboard, "velocity", cfg.keyboard.velocity);
  cfg.keyboard.passive = text(keyboard, "passive", cfg.keyboard.passive);
  cfg.keyboard.estop = text(keyboard, "estop", cfg.keyboard.estop);
  cfg.keyboard.rearm = text(keyboard, "rearm", cfg.keyboard.rearm);
  cfg.keyboard.slow_toggle =
      text(keyboard, "slow_toggle", cfg.keyboard.slow_toggle);
  cfg.keyboard.vx_step = number(keyboard, "vx_step", cfg.keyboard.vx_step);
  cfg.keyboard.vy_step = number(keyboard, "vy_step", cfg.keyboard.vy_step);
  cfg.keyboard.wz_step = number(keyboard, "wz_step", cfg.keyboard.wz_step);
  cfg.keyboard.slow_scale =
      number(keyboard, "slow_scale", cfg.keyboard.slow_scale);
  return cfg;
}

}  // namespace

Go2RuntimeBridge::Go2RuntimeBridge(const YAML::Node& cfg,
                                   const runtime::CommandLimits& limits,
                                   const GamepadAdapter::Expressions& expressions,
                                   Callbacks callbacks)
    : runtime_(makeConfig(cfg, limits)),
      limits_(limits),
      adapter_(expressions),
      callbacks_(std::move(callbacks)),
      names_{{"Passive", runtime::Behavior::kPassive},
             {"FixStand", runtime::Behavior::kStand},
             {"LieDown", runtime::Behavior::kLie},
             {"Velocity", runtime::Behavior::kVelocity}} {
  if (callbacks_.state_id) {
    for (const auto& entry : names_) {
      try {
        id_to_behavior_.emplace(callbacks_.state_id(entry.first), entry.second);
      } catch (const std::exception&) {
        // 该机器人未配置这个状态，忽略
      }
    }
  }
}

void Go2RuntimeBridge::tick() {
  if (!FSMState::lowstate) return;
  const auto& state = FSMState::lowstate;

  runtime::RobotSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    const auto& imu = state->msg_.imu_state();
    const auto& quaternion = imu.quaternion();
    const Eigen::Quaternionf q(quaternion[0], quaternion[1], quaternion[2],
                               quaternion[3]);
    const Eigen::Vector3f gravity =
        q.conjugate() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    snapshot.roll = std::atan2(-gravity.y(), -gravity.z());
    snapshot.pitch =
        std::atan2(gravity.x(), std::hypot(gravity.y(), gravity.z()));
    const auto& gyro = imu.gyroscope();
    snapshot.gyro_norm =
        std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    float max_joint_vel = 0.0f;
    const auto& motors = state->msg_.motor_state();
    for (std::size_t i = 0; i < motors.size(); ++i) {
      max_joint_vel = std::max(max_joint_vel, std::fabs(motors[i].dq()));
    }
    snapshot.max_joint_vel = max_joint_vel;
  }

  const std::string current_name =
      callbacks_.current_state_name ? callbacks_.current_state_name()
                                    : std::string();
  const std::optional<runtime::Behavior> current = behaviorFromName(current_name);
  if (current) runtime_.setCurrentBehavior(*current);

  runtime::KeyPulse key;
  if (FSMState::keyboard) {
    key.key = FSMState::keyboard->key();
    key.just_pressed = FSMState::keyboard->on_pressed;
  }

  const runtime::ActionSet pad_actions = adapter_.actions(state->joystick);
  runtime::MotionCommand pad_motion;
  pad_motion.vx =
      scaleAxis(state->joystick.ly(), limits_.min_vx, limits_.max_vx);
  pad_motion.vy =
      scaleAxis(-state->joystick.lx(), limits_.min_vy, limits_.max_vy);
  pad_motion.wz =
      scaleAxis(-state->joystick.rx(), limits_.min_wz, limits_.max_wz);

  const runtime::TickOutput output =
      runtime_.tick(nowSeconds(), snapshot, pad_actions, pad_motion,
                    !state->isJoystickTimeout(), key);

  if (current && output.behavior != *current && callbacks_.request_state) {
    const std::optional<int> id = stateId(output.behavior);
    if (id) callbacks_.request_state(*id);
  }
}

bool Go2RuntimeBridge::allowTransition(int fsm_id) const {
  const auto it = id_to_behavior_.find(fsm_id);
  if (it == id_to_behavior_.end()) return true;
  return runtime_.allowTransition(it->second);
}

std::optional<runtime::Behavior> Go2RuntimeBridge::behaviorFromName(
    const std::string& name) const {
  const auto it = names_.find(name);
  if (it == names_.end()) return std::nullopt;
  return it->second;
}

std::optional<int> Go2RuntimeBridge::stateId(runtime::Behavior behavior) const {
  for (const auto& entry : id_to_behavior_) {
    if (entry.second == behavior) return entry.first;
  }
  return std::nullopt;
}
```

- [ ] **Step 3: 改 `main.cpp`**

```cpp
// deploy/robots/go2/main.cpp
#include <filesystem>
#include <memory>
#include <string>

#include "FSM/CtrlFSM.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_LieDown.h"
#include "FSM/State_Passive.h"
#include "FSM/State_RLBase.h"
#include "GamepadAdapter.h"
#include "Go2RuntimeBridge.h"
#include "runtime/ticker.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

namespace {

std::string text(const YAML::Node& node, const char* key,
                 const std::string& fallback) {
  return node && node[key] ? node[key].as<std::string>() : fallback;
}

runtime::CommandLimits loadLimits(const YAML::Node& runtime_cfg) {
  runtime::CommandLimits limits;
  const std::string relative = text(
      runtime_cfg, "velocity_params",
      "config/policy/velocity/v0/params/deploy.yaml");
  const auto path = param::proj_dir / relative;
  if (!std::filesystem::exists(path)) return limits;

  const YAML::Node deploy = YAML::LoadFile(path.string());
  const YAML::Node ranges = deploy["commands"]["base_velocity"]["ranges"];
  if (!ranges) return limits;

  limits.min_vx = ranges["lin_vel_x"][0].as<float>();
  limits.max_vx = ranges["lin_vel_x"][1].as<float>();
  limits.min_vy = ranges["lin_vel_y"][0].as<float>();
  limits.max_vy = ranges["lin_vel_y"][1].as<float>();
  limits.min_wz = ranges["ang_vel_z"][0].as<float>();
  limits.max_wz = ranges["ang_vel_z"][1].as<float>();
  return limits;
}

GamepadAdapter::Expressions loadExpressions() {
  const YAML::Node gamepad = param::config["Runtime"]["gamepad"];
  return {text(gamepad, "stand", ""),    text(gamepad, "lie", ""),
          text(gamepad, "velocity", ""), text(gamepad, "passive", ""),
          text(gamepad, "estop", ""),    text(gamepad, "rearm", "")};
}

}  // namespace

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n";

    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);

    const YAML::Node runtime_cfg = param::config["Runtime"];
    Go2RuntimeBridge::Callbacks callbacks;
    callbacks.request_state = [&fsm](int id) { fsm->requestState(id); };
    callbacks.current_state_name = [&fsm]() { return fsm->currentStateName(); };
    callbacks.state_id = [](const std::string& name) {
        return FSMStringMap.right.at(name);
    };

    auto runtime_bridge = std::make_shared<Go2RuntimeBridge>(
        runtime_cfg, loadLimits(runtime_cfg), loadExpressions(), callbacks);

    fsm->setTransitionGate(
        [runtime_bridge](int id) { return runtime_bridge->allowTransition(id); });
    FSMState::runtime = runtime_bridge;

    fsm->start();

    std::cout << "Keyboard: [1] stand [2] lie [0] passive [x] estop [r] rearm\n";
    std::cout << "Gamepad: LT+up stand, LT+down lie, LT+B passive, RT+B estop, LT+right rearm\n";

    while (true)
    {
        sleep(1);
    }

    return 0;
}
```

- [ ] **Step 4: 构建**

```bash
P=$HOME/unitree_sdk2/install
cmake -S deploy/robots/go2 -B build/go2 \
  -DCMAKE_CXX_FLAGS="-I$P/include -I$P/include/ddscxx" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$P/lib -Wl,-rpath,$P/lib"
cmake --build build/go2 -j
```

Expected: 编译通过，无错误。若报 `unitree_joystick_dsl.hpp` 找不到，确认 `-I deploy/include` 已包含（`deploy/robots/go2/CMakeLists.txt` 已有）。

- [ ] **Step 5: 提交**

```bash
git add deploy/robots/go2/include deploy/robots/go2/src/Go2RuntimeBridge.cpp deploy/robots/go2/main.cpp
git commit -m "feat: wire go2 runtime bridge into fsm"
```

---

### Task 10: README 与最终验证

**Files:**
- Create: `deploy/robots/go2/README.md`

**Interfaces:**
- Consumes: 前述全部
- Produces: 使用文档 + 最终验证记录

- [ ] **Step 1: 写 README**

```markdown
# Go2 runtime skeleton

## Build

The SDK is installed to a local prefix (sudo is unavailable in this container):

```bash
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
cmake -S ~/unitree_sdk2 -B ~/unitree_sdk2/build \
  -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$HOME/unitree_sdk2/install
cmake --build ~/unitree_sdk2/build -j
cmake --install ~/unitree_sdk2/build

P=$HOME/unitree_sdk2/install
cmake -S deploy/robots/go2 -B build/go2 \
  -DCMAKE_CXX_FLAGS="-I$P/include -I$P/include/ddscxx" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$P/lib -Wl,-rpath,$P/lib"
cmake --build build/go2 -j
```

Run (on a machine with a robot or unitree_mujoco, gamepad optional):

```bash
cd build/go2 && ./go2_ctrl --network=lo
```

## Keyboard

| Key | Action |
|---|---|
| `1` | stand |
| `2` | lie down |
| `0` | passive |
| `3` | velocity (disabled until a policy is installed) |
| `x` | software E-stop (latch) |
| `r` | re-arm (needs 0.5 s of safe state) |
| `w/s` | vx +/- 0.1 m/s |
| `a/d` | vy +/- 0.1 m/s |
| `q/e` | wz +/- 0.2 rad/s |
| `space` | zero command |
| `z` | toggle slow mode (x0.4) |

## Gamepad (DSL)

| Action | Expression |
|---|---|
| stand | `LT + up.on_pressed` |
| lie | `LT + down.on_pressed` |
| passive | `LT + B.on_pressed` |
| estop | `RT + B.on_pressed` |
| rearm | `LT + right.on_pressed` |

## Safety

- DEGRADED (tilt > 0.35 rad, gyro > 3.5 rad/s, joint vel > 30 rad/s): command zeroed, posture held, auto-recovers after 0.5 s.
- CRITICAL (tilt > 0.60 rad, gyro > 6 rad/s, joint vel > 50 rad/s, NaN/jump, E-stop): command zeroed, transition to Passive, latched.
- Re-arm only in Passive after 0.5 s of OK; then press `1` (stand) explicitly.
- Thresholds live under `Runtime.safety` in `config/config.yaml`.

## Tests

```bash
cmake -S deploy/tests -B build/tests
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure
```
```

- [ ] **Step 2: 全量验证**

```bash
cmake -S deploy/tests -B build/tests
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure

P=$HOME/unitree_sdk2/install
cmake -S deploy/robots/go2 -B build/go2 \
  -DCMAKE_CXX_FLAGS="-I$P/include -I$P/include/ddscxx" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$P/lib -Wl,-rpath,$P/lib"
cmake --build build/go2 -j
```

Expected: ctest 全通过；`go2_ctrl` 链接成功。

- [ ] **Step 3: 提交**

```bash
git add deploy/robots/go2/README.md
git commit -m "docs: document go2 runtime skeleton build and controls"
```

---

## 自检结果

- **Spec 覆盖**：输入抽象（Task 2/9）、仲裁（Task 3/6）、安全（Task 4/6）、行为/姿态（Task 5/6/8）、FSM 钩子（Task 7）、接线（Task 9）、测试与文档（Task 1–6/10）。
- **类型一致性**：`KeyboardMap`、`CommandMux`、`SafetySupervisor`、`BehaviorRouter`、`Runtime` 的签名在各任务间一致；`velocity_enabled` 贯穿 `RuntimeConfig` 与 `BehaviorRouter::update`。
- **无占位符**：所有代码步骤均为完整实现，测试均为可运行断言。
- **已知边界**：Task 8/9 的编译验证依赖 `$HOME/unitree_sdk2/install`；`simulate/` 与端到端运行不在本轮范围。
