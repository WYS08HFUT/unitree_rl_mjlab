// deploy/robots/go2/include/GamepadAdapter.h
#pragma once

#include <exception>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>

#include <spdlog/spdlog.h>

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

  // Resolve every key name in the AST. A single evaluation is not enough:
  // "&&"/"||" short-circuit, so an unknown key on the right-hand side of a
  // false "LT + ..." atom is never touched at idle and would still throw
  // later, inside the FSM thread, once LT is pressed.
  static void validateKeys(const unitree::common::dsl::Node& node,
                           const unitree::common::UnitreeJoystick& probe) {
    using unitree::common::dsl::Node;
    switch (node.kind) {
      case Node::kAtom:
        (void)unitree::common::dsl::GetKey(probe, node.atom.name);
        break;
      case Node::kNot:
        if (node.lhs) validateKeys(*node.lhs, probe);
        break;
      case Node::kAnd:
      case Node::kOr:
        if (node.lhs) validateKeys(*node.lhs, probe);
        if (node.rhs) validateKeys(*node.rhs, probe);
        break;
    }
  }

  void add(const std::string& name, const std::string& expression) {
    if (expression.empty()) return;
    try {
      unitree::common::dsl::Parser parser(expression);
      auto ast = parser.Parse();
      Predicate predicate = unitree::common::dsl::Compile(*ast);
      // Validate once, at construction, instead of throwing every tick
      // inside the FSM thread.
      unitree::common::UnitreeJoystick probe;
      validateKeys(*ast, probe);
      (void)predicate(probe);
      predicates_[name] = std::move(predicate);
    } catch (const std::exception& error) {
      spdlog::warn("Runtime.gamepad.{} disabled (bad expression '{}'): {}",
                   name, expression, error.what());
    }
  }

  bool eval(const std::string& name,
            const unitree::common::UnitreeJoystick& joystick) const {
    const auto it = predicates_.find(name);
    return it != predicates_.end() && it->second(joystick);
  }

  std::unordered_map<std::string, Predicate> predicates_;
};
