#pragma once

namespace runtime {

class IRuntimeTicker {
 public:
  virtual ~IRuntimeTicker() = default;
  virtual void tick() = 0;
  virtual bool allowTransition(int fsm_id) const = 0;
};

}  // namespace runtime
