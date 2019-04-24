// Author: Tucker Haydon

#include "quad_state_guard.h"

namespace mediation_layer {
  void QuadStateGuard::Write(const QuadState& state) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->state_ = state;
  }

  void QuadStateGuard::Read(QuadState& state) const {
    std::lock_guard<std::mutex> lock(this->mtx_);
    state = this->state_;
  }
}
