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
