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
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

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

    // 键盘在 --help/--version 之后创建，避免静态析构阶段终止（Keyboard 线程未被 join）
    FSMState::keyboard = std::make_shared<Keyboard>();

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
