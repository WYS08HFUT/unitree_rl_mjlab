// deploy/robots/go2/include/Go2RuntimeConfig.h
#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

#include "runtime/types.h"

namespace go2 {

inline float number(const YAML::Node& node, const char* key, float fallback) {
  return node && node[key] ? node[key].as<float>() : fallback;
}

inline bool boolean(const YAML::Node& node, const char* key, bool fallback) {
  return node && node[key] ? node[key].as<bool>() : fallback;
}

inline std::string text(const YAML::Node& node, const char* key,
                        const std::string& fallback) {
  return node && node[key] ? node[key].as<std::string>() : fallback;
}

// The terminal keyboard reports the space bar as " ", which YAML cannot express
// readably, so configs spell that one binding as the alias "space".
inline std::string keyName(const YAML::Node& node, const char* key,
                           const std::string& fallback) {
  const std::string value = text(node, key, fallback);
  return value == "space" ? std::string(" ") : value;
}

inline runtime::RuntimeConfig makeRuntimeConfig(
    const YAML::Node& root, const runtime::CommandLimits& limits) {
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

  // Every keyboard binding is configurable: mode keys, motion keys, the
  // zero-command key and the slow-mode toggle plus its step sizes.
  const YAML::Node keyboard = root["keyboard"];
  cfg.keyboard.stand = text(keyboard, "stand", cfg.keyboard.stand);
  cfg.keyboard.lie = text(keyboard, "lie", cfg.keyboard.lie);
  cfg.keyboard.velocity = text(keyboard, "velocity", cfg.keyboard.velocity);
  cfg.keyboard.passive = text(keyboard, "passive", cfg.keyboard.passive);
  cfg.keyboard.estop = text(keyboard, "estop", cfg.keyboard.estop);
  cfg.keyboard.rearm = text(keyboard, "rearm", cfg.keyboard.rearm);
  cfg.keyboard.vx_up = text(keyboard, "vx_up", cfg.keyboard.vx_up);
  cfg.keyboard.vx_down = text(keyboard, "vx_down", cfg.keyboard.vx_down);
  cfg.keyboard.vy_up = text(keyboard, "vy_up", cfg.keyboard.vy_up);
  cfg.keyboard.vy_down = text(keyboard, "vy_down", cfg.keyboard.vy_down);
  cfg.keyboard.wz_up = text(keyboard, "wz_up", cfg.keyboard.wz_up);
  cfg.keyboard.wz_down = text(keyboard, "wz_down", cfg.keyboard.wz_down);
  cfg.keyboard.stop = keyName(keyboard, "stop", cfg.keyboard.stop);
  cfg.keyboard.slow_toggle =
      text(keyboard, "slow_toggle", cfg.keyboard.slow_toggle);
  cfg.keyboard.vx_step = number(keyboard, "vx_step", cfg.keyboard.vx_step);
  cfg.keyboard.vy_step = number(keyboard, "vy_step", cfg.keyboard.vy_step);
  cfg.keyboard.wz_step = number(keyboard, "wz_step", cfg.keyboard.wz_step);
  cfg.keyboard.slow_scale =
      number(keyboard, "slow_scale", cfg.keyboard.slow_scale);
  return cfg;
}

}  // namespace go2
