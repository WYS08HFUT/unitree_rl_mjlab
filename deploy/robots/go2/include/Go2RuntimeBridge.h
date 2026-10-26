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
