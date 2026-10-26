# Go2 运行时骨架设计（输入抽象 / 命令仲裁 / 安全监督 / 姿态切换）

- 日期：2026-10-26
- 状态：待评审
- 适用仓库：`unitree_rl_mjlab`（分支 `main`）
- 上游计划：`plan.md` 的里程碑 2（游戏式遥控：键盘/手柄、姿态切换、状态机、命令仲裁）
- 载体：扩展现有 C++ 部署链路（`deploy/`），不新建 Python runtime

## 1. 背景与目标

`plan.md` 提出的目标架构包含输入层、行为状态机与命令仲裁、安全监督、感知/导航、RL 策略五层。当前仓库已有：

- `deploy/include/FSM`：配置驱动的 C++ 状态机，支持 `LT + up.on_pressed` 这类 DSL 组合键
- `deploy/include/isaaclab`：ONNX 推理、关节动作、观测，速度指令直接取自手柄 `lowstate->joystick`
- `deploy/include/isaaclab/devices/keyboard`：终端键盘，go2 当前未启用（`main.cpp` 中为 `nullptr`）
- `deploy/robots/go2`：Passive → FixStand → Velocity 三状态，无独立安全监督与命令仲裁

本轮（里程碑 2 的第一刀）交付**运行时骨架**：把"游戏式输入、命令仲裁、安全监督、姿态切换"变成可独立测试的组件，并在 go2 上接线。**不训练策略、不做感知/导航、不改其它机器人**。

## 2. 已确认的决策

| 决策 | 选择 |
|---|---|
| 范围 | 运行时骨架（输入抽象 + 行为/仲裁 + 安全监督 + 姿态切换） |
| 载体 | 扩展现有 C++ 部署链路 |
| 策略 | 本轮不训练；Velocity 仅占位，行走验证后置 |
| 安全响应 | 分级响应（DEGRADED / CRITICAL）+ CRITICAL 锁存需人工复位 |
| 架构 | 运行时段包在现有 FSM 外层；核心纯逻辑可单测，适配层依赖 SDK |
| 验证 | 纯 C++ 单测 + 离线回放；仿真/真机联调在装有 unitree_sdk2 的机器执行 |

## 3. 非目标

- 策略训练、ONNX 导出、unitree_mujoco 走跑联调
- 目标跟随、局部避障、导航（仅预留 `AUTO` 命令源）
- Posture RL 策略（站立/卧下用脚本化插值）
- 修改其他机器人（g1、h1_2、a2、as2、r1、h2）的配置或行为
- 安装 unitree_sdk2 / cyclonedds / glfw

## 4. 架构

### 4.1 分层

```
deploy/include/runtime/          # 纯核心：仅 C++ 标准库，无 SDK 依赖，可单测
  types.h           MotionCommand / Behavior / SafetyLevel / ActionSet / RobotSnapshot
  ticker.h          IRuntimeTicker 抽象：共享 FSMState 只依赖此接口
  keyboard_map.h    键盘点按 → ActionSet 与速度设定值
  command_mux.h     仲裁、限幅、死区、斜率、输入超时
  safety.h          检查项、分级、锁存、恢复与复位
  behavior.h        ActionSet + SafetyStatus → 目标 Behavior
  runtime.h         一次 tick 的编排，持有上述对象

deploy/include/FSM/State_LieDown.h   # 新增脚本化卧下状态

deploy/robots/go2/include/           # 适配层（依赖 unitree_sdk2），仅 go2
  GamepadAdapter.h    用现有 DSL 对 lowstate->joystick 求值 ActionSet，读取模拟轴
  Go2RuntimeBridge.h  读 YAML、构造 Runtime、Behavior→FSM id、虚拟 joystick、安全门

deploy/robots/go2/src/               # 对应 .cpp（CMake 用 GLOB_RECURSE src/*.cpp 收集）
```

核心不 include 任何 `unitree/` 头文件；适配层才 include `Types.h`、`unitree_joystick_dsl.hpp`。

### 4.2 每个控制周期

控制周期沿用 `CtrlFSM` 的 1 kHz 循环，**不新增线程**。

```
FSMState::pre_run()
  1. lowstate->update()                     # 现有
  2. keyboard->update()                     # 现有
  3. runtime_bridge->tick()                 # 新增；bridge 自己读取 now/LowState/键盘
       0. 同步：从 FSM 当前状态字符串反查 Behavior，setCurrentBehavior()（防止请求被 gate 拒绝后状态漂移）
       a. 采集：物理手柄按钮/轴、键盘按键、LowState（姿态/角速度/关节）
       b. 预检：SafetySupervisor 评估当前状态与上一周期命令
       c. 动作：GamepadAdapter(DSL) + KeyboardMap → ActionSet
       d. 路由：BehaviorRouter → 目标 Behavior（或保持）
       e. 仲裁：CommandMux → MotionCommand
       f. 输出：behavior / command / safety；写虚拟 joystick 的轴
  4. 如目标 Behavior 变化 → requestState(fsm_id)（经安全门）

CtrlFSM::run_()
  1. pre_run() / run() / post_run()
  2. 优先处理 requestState 的挂起请求（经 transition gate）
  3. 否则执行现有 registered_checks（含 lowstate 超时 → Passive）
```

`requestState` 在本周期 `post_run()` 后生效，最多延迟 1 ms。

### 4.3 数据所有权

- `Runtime` 持有核心状态（锁存、计时、命令目标、安全等级）；不持有 SDK 对象
- `GamepadAdapter` 只读 `lowstate->joystick`，不修改
- `Go2RuntimeBridge` 实现 `IRuntimeTicker`，持有 `Runtime`、虚拟 `UnitreeJoystick`（仅供 `State_RLBase` 的观测读取命令），以及 `Behavior → FSM id` 映射
- `FSMState` 新增静态 `std::shared_ptr<IRuntimeTicker>`（与 `lowcmd`/`keyboard` 同样式），所有机器人默认为 `nullptr`；共享层因此不需要 include 任何 go2 头文件

## 5. 核心接口

```cpp
// types.h
enum class Behavior { kPassive, kStand, kLie, kVelocity };
enum class SafetyLevel { kOk, kDegraded, kCritical };
enum class CommandSource { kManual, kAuto, kSafety };

struct MotionCommand { float vx{0}; float vy{0}; float wz{0}; };
struct RobotSnapshot {
  float roll{0}, pitch{0};          // rad，由四元数换算，适配层填充
  float gyro_norm{0};               // rad/s
  bool  joint_limits_ok{true};
  bool  action_finite{true};
  bool  action_jump{false};         // 由动作层判定后传入
};
struct ActionSet {
  bool stand{false}, lie{false}, velocity{false}, passive{false};
  bool estop{false}, rearm{false};
};
struct TickOutput {
  Behavior behavior;
  MotionCommand command;
  SafetyLevel safety;
  bool latched;
};
struct KeyPulse { std::string key; bool just_pressed{false}; };  // 来自 Keyboard
```

```cpp
// runtime.h
class Runtime {
 public:
  TickOutput tick(double now, const RobotSnapshot& robot,
                  const ActionSet& gamepad_actions,
                  const MotionCommand& gamepad_motion,
                  const KeyPulse& key);                  // 时间/输入由外部注入，可回放
  bool allowTransition(Behavior target) const;           // 供 CtrlFSM gate 使用
  void setCurrentBehavior(Behavior actual);              // 每 tick 用 FSM 实际状态校准
  const MotionCommand& command() const;
 private:
  RuntimeConfig cfg_;
  SafetySupervisor safety_;
  BehaviorRouter router_;
  CommandMux mux_;
  KeyboardMap keyboard_;
  // 锁存/计时/命令目标等状态
};
```

```cpp
// ticker.h：共享层唯一看到的接口，避免 FSMState 依赖 go2/SDK
class IRuntimeTicker {
 public:
  virtual ~IRuntimeTicker() = default;
  virtual void tick() = 0;                             // FSMState::pre_run 调用
  virtual bool allowTransition(int fsm_id) const = 0;  // CtrlFSM gate 调用
};
```

`tick` 内不读取系统时钟，全部用传入的 `now`，保证回放测试可重复。

## 6. 行为、动作与仲裁

### 6.1 行为

| Behavior | FSM 状态 | 说明 |
|---|---|---|
| `kPassive` | `Passive` | 阻尼，安全/急停落点 |
| `kStand` | `FixStand` | 脚本化起立并保持 |
| `kLie` | `LieDown` | 脚本化卧下并保持（新增） |
| `kVelocity` | `Velocity` | ONNX 策略（本轮无策略，仅占位） |

### 6.2 动作优先级

同一 tick 多个动作同时触发时：`estop > rearm > passive > lie > stand > velocity`。

- `estop`：进入 `kPassive` 并锁存
- `rearm`：仅在锁存、当前为 `kPassive`、安全连续 `kOk` ≥ 0.5 s 时清除锁存（仍停留在 `kPassive`）；未锁存时忽略
- 锁存期间除 `rearm` 外的动作全部忽略（gate 拒绝所有非 `kPassive` 目标）
- 非锁存时：`passive/lie/stand/velocity` 直接切换；`kVelocity` 还要求 `policy.onnx` 存在（见 7.3），否则忽略并 warning

### 6.3 键盘映射

go2 的 `main.cpp` 需从 `nullptr` 改为 `std::make_shared<Keyboard>()`（参考 `deploy/robots/g1/main.cpp`）。

| 键 | 动作/含义 |
|---|---|
| `1` | stand |
| `2` | lie |
| `3` | velocity |
| `0` | passive |
| `x` | estop |
| `r` | rearm |
| `w` / `s` | vx +step / -step |
| `a` / `d` | vy +step / -step |
| `q` / `e` | wz +step / -step |
| `space` | 命令清零 |
| `z` | 切换慢速模式（步长 × `slow_scale`） |

键盘用**点按设定值**而不是压键：终端没有可靠的 key-up，`Keyboard` 只能可靠给出"当前字符 + 上升沿"。`w` 每次点按使 vx 目标增加 `vx_step`，`space` 清零；目标值随仲裁限幅。空格在 `Keyboard` 里是字符 `" "`，配置中用别名 `space` 表示。

> 修正说明：初稿用 `shift` 作为慢速修饰键，但终端无法区分"按住 shift"与"大写字母"，因此改为 `z` 切换慢速模式（可单测、可显示状态）。

### 6.4 手柄映射（DSL 表达式）

复用 `unitree_joystick_dsl.hpp`，表达式对 `lowstate->joystick` 求值，由 `GamepadAdapter` 编译一次、每 tick 调用。

| 动作 | 表达式（默认，沿用现有覆盖） |
|---|---|
| stand | `LT + up.on_pressed` |
| lie | `LT + down.on_pressed` |
| velocity | `RT + A.on_pressed` |
| passive | `LT + B.on_pressed` |
| estop | `RT + B.on_pressed` |
| rearm | `LT + right.on_pressed` |

模拟轴（`lx/ly/rx`）直接作为手动速度输入：`vx = ly`、`vy = -lx`、`wz = -rx`（与现有观测约定一致）。

### 6.5 CommandMux

- 输入：手动命令（手柄轴 / 键盘设定值）、安全等级
- 优先级：`kSafety`（全零）> `kAuto`（本轮恒零）> `kManual`
- 限幅：`lin_vel_x`、`lin_vel_y`、`ang_vel_z`，由适配层从 `deploy.yaml` 的 `commands.base_velocity.ranges` 读取后传入
- 死区：0.05；斜率：2.0 m/s²、4.0 rad/s²
- 输入超时：仅作用于**流式来源**（手柄轴、未来的 AUTO 命令）；键盘设定值不会因"没有新按键"而超时，只能用 `space` 清零
- 仅在 `kVelocity` 输出非零；`kPassive/kStand/kLie` 恒零

### 6.6 命令写入策略观测

`State_RLBase` 的观测从 `env->robot->data.joystick` 读速度。做法：

1. bridge 持有一个 `UnitreeJoystick virtual_joystick_`，将 `lx/ly/rx` 的 `smooth` 设为 1.0、`deadzone` 设为 0.0
2. 每 tick 用仲裁后的物理命令按 `deploy.yaml` 限幅归一化后 `extract(...)`：`ly = vx / max_vx`、`lx = -vy / max_vy`、`rx = -wz / max_wz`（与现有观测 `vx=ly, vy=-lx, wz=-rx` 的符号约定一致）
3. `State_RLBase` 新增 `enter()/exit()`：`enter()` 把 `env->robot->data.joystick` 指向该对象；`exit()` 还原为构造时保存的原始指针

这样不需要修改共享的 `observations.h`。

## 7. 安全监督

### 7.1 检查与阈值（默认值，全部可配）

| 检查 | DEGRADED | CRITICAL |
|---|---|---|
| roll/pitch 绝对值 | > 0.35 rad | > 0.60 rad |
| 角速度范数 | > 3.5 rad/s | > 6.0 rad/s |
| 关节速度绝对值（任一） | > 30 rad/s | > 50 rad/s |
| 指令时延（AUTO 预留） | > 0.10 s | > 0.30 s |
| 动作 NaN / 单周期跳变 | — | NaN 或 \|Δq\| > 1.5 rad |
| 软急停动作 | — | 触发 |
| LowState 超时（现有） | — | 现有 `isTimeout()` 已在 FSM 中处理 |

`RobotSnapshot` 由适配层填充：四元数→roll/pitch、`lowstate` 角速度范数、关节速度、动作层给出的 NaN/跳变标志。

- **指令时延**只对未来的 AUTO 命令源有意义：手动命令每 tick 重新生成，age 恒为 0。本轮该检查保留但不触发，作为接入感知/导航前的占位。
- **动作跳变**由 `State_RLBase` 侧计算：保存上一 tick 的关节目标，任一关节非有限值或 `|Δq| > action_jump_critical` 即置 `action_jump=true`；该标志由 bridge 在下一 tick 的 `RobotSnapshot` 中传入。
- 阈值为初始值，全部来自 YAML，允许现场覆盖。

### 7.2 响应

- **DEGRADED**：命令立即归零（含斜率限制器状态清零），保持当前 Behavior 与姿态；当所有检查连续 0.5 s 低于 DEGRADED 阈值（即回到 `kOk`）后自动恢复；每个等级进入时记一条日志
- **CRITICAL**：命令归零，请求 `kPassive`，置锁存；锁存只能由 `rearm` 清除（条件见 6.2）
- `State_RLBase` 现有的 `bad_orientation(1.0)` 保留为最后一道兜底，阈值与 supervisor 不冲突
- 锁存状态写入虚拟 joystick 时命令恒零，确保策略侧不会收到残余速度

### 7.3 与 FSM 的配合

- `CtrlFSM::requestState(int)`：记录挂起目标；`run_()` 在 `post_run()` 之后、`registered_checks` 之前处理，目标相同或 gate 拒绝时丢弃。只允许 FSM 线程调用（`pre_run` 内），不加锁。
- `CtrlFSM::currentStateName()`：返回当前状态名，供 bridge 每 tick 校准 `setCurrentBehavior()`。
- `CtrlFSM::setTransitionGate(fn)`：所有切换（挂起请求与 `registered_checks`）前调用；go2 的 gate 为 `runtime->allowTransition(target)`
- `allowTransition(PASSIVE)` 恒真；锁存或 `kCritical` 时其余目标为假；`kDegraded` 不阻止切换但命令已归零
- 现有 DSL `transitions` 在 go2 配置中清空，统一由运行时请求；其他机器人配置不动，行为不变
- bridge 启动时用 `FSMStringMap` 把 Behavior 解析为状态 id；某个状态缺失（例如未配置 `LieDown`）时禁用对应动作并打 warning，而不是运行时崩溃
- `main.cpp` 构造 bridge 后注入两个回调：`request_state(int)` 与 `current_state_name()`，并把 FSM 的 gate 设为 `bridge.allowTransition`；bridge 因此不直接依赖 `CtrlFSM` 类型
- `kVelocity` 还要求对应 `policy.onnx` 存在；缺失时忽略 `velocity` 动作并 warning（避免进入会抛异常的 `State_RLBase` 构造）

## 8. 新增 State_LieDown

结构复制 `State_FixStand`，配置独立：

```yaml
FSM:
  _:
    LieDown: { id: 4 }
  LieDown:
    transitions: {}
    kp: [60, 80, 80, 60, 80, 80, 60, 80, 80, 60, 80, 80]
    kd: [5, 4, 4, 5, 4, 4, 5, 4, 4, 5, 4, 4]
    ts: [0.0, 1.5]
    qs:
      - []                                  # enter() 时替换为当前关节角
      - [0.0, 1.45, -2.70, 0.0, 1.45, -2.70,
         0.0, 1.45, -2.70, 0.0, 1.45, -2.70]
```

目标姿态的取值依据 Go2 关节范围（hip ±1.0472、front thigh [-1.5708, 3.4907]、rear thigh [-0.5236, 4.5379]、calf [-2.7227, -0.83776]），取保守的"收腿伏地"值，均在范围内；这是**初始标定值**，需在仿真/真机确认后再调。

- `enter()`：设置 `kp/kd`，`q0 = 当前 q`，`t0 = now`
- `run()`：线性插值到目标并保持
- 从 `LieDown` 回 `FixStand` 时，`FixStand::enter()` 会以当前 q 为起点插值到站立姿态，现有实现已支持
- `LieDown` 的 `qs` 目标值需在 go2 上实测标定，本轮先给出保守的"收腿伏地"姿态，并在 README 标注待标定

## 9. 配置

在 `deploy/robots/go2/config/config.yaml` 增加：

```yaml
Runtime:
  command:
    deadzone: 0.05
    accel_limit: 2.0
    yaw_accel_limit: 4.0
    stale_timeout: 0.10
  safety:
    tilt_degraded: 0.35
    tilt_critical: 0.60
    gyro_degraded: 3.5
    gyro_critical: 6.0
    joint_vel_degraded: 30.0
    joint_vel_critical: 50.0
    command_age_degraded: 0.10
    command_age_critical: 0.30
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

并清空 go2 各状态的 `transitions`（保留 `FixStand` 的 `kp/kd/ts/qs`，新增 `LieDown`）。

`deploy.yaml` 的观测限幅仍是速度上限的唯一来源；bridge 读取后传给 `RuntimeConfig`，避免两处配置漂移。

## 10. 测试

### 10.1 结构

- `deploy/tests/CMakeLists.txt`：独立 CMake 工程，C++17，不链接 unitree_sdk2 / DDS / onnxruntime
- 轻量断言（`CHECK/REQUIRE` 宏 + 失败计数），`main` 返回非零即失败；`ctest` 注册用例
- 每个核心模块一个测试文件，另有 `test_replay.cpp` 驱动整个 `Runtime`

### 10.2 用例矩阵

| 文件 | 覆盖 |
|---|---|
| `test_keyboard_map.cpp` | 动作键、未知键忽略、步长累加与限幅、慢速切换、space 清零 |
| `test_command_mux.cpp` | 优先级、限幅、死区、斜率、超时归零、非 Velocity 输出零 |
| `test_safety.cpp` | 各阈值边界、DEGRADED 自动恢复、CRITICAL 锁存、NaN、指令时延、抖动不误触发 |
| `test_behavior.cpp` | 动作优先级、锁存拒绝 stand/velocity、rearm 时序、rearm 后仍需显式 stand |
| `test_replay.cpp` | 时间线回放：`Passive → Stand → 倾角 DEGRADED → 恢复 → Velocity → 软急停 → 锁存 → rearm → Stand`，以及 `Stand → Lie → Stand`、过期 rearm 被拒 |

回放用例使用虚拟时间（每次 tick 传入递增 `now`），不依赖真实时钟或 SDK。

### 10.3 运行

```bash
cmake -S deploy/tests -B build/tests
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure
```

## 11. 验收标准（DoD）

1. `ctest` 全部通过，贴出实际输出
2. go2 接线完成：`main.cpp`、`FSMState`/`CtrlFSM` 钩子、`State_RLBase` 指针切换、`State_LieDown`、YAML 配置
3. 适配层文件 ≤ ~150 行且显式列出依赖的 SDK 头文件，便于在装有 SDK 的机器首次编译
4. 其他机器人零影响：未改它们的配置；共享类新增成员在 bridge 为 `nullptr` 时行为不变
5. `deploy/robots/go2/README.md`：按键表、安全行为、锁存/复位流程、仿真/真机启动步骤
6. 按 TDD 小步提交

## 12. 风险与限制

- **无法本机编译 go2_ctrl**：缺 unitree_sdk2/cyclonedds/glfw；适配层与接线只能静态审阅，首次编译可能暴露 include/API 细节问题
- **无策略**：本轮 Velocity 只是占位，行走能力未验证
- **键盘语义**：终端无 key-up，采用点按设定值；长按/组合键由手柄承担
- **阈值需实测**：0.35/0.60 rad 等为初始值，真机需重新标定
- **LieDown 姿态未标定**：首版用保守收腿姿态，需在仿真/真机确认

## 13. 后续工作

1. 在装有 SDK 的机器编译 go2_ctrl，接入 ONNX 策略，在 unitree_mujoco 完成走跑与安全联调
2. 训练/导出 Go2 Flat 策略（`src/tasks/velocity/config/go2`）
3. 里程碑 3：`AUTO` 命令源接入目标跟随 + 局部避障，复用本设计的仲裁与安全门
4. 里程碑 4：真机分阶段验证（站立 → 低速 → 行走 → 技能切换）
