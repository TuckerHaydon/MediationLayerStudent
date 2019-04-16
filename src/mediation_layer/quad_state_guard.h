// Author: Tucker Haydon

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "quad_state.h"

namespace mediation_layer {
  // State guard is a wrapper around a QuadState that ensures thread-safe,
  // sychronized access to the QuadState. 
  class QuadStateGuard {
    private:
      QuadState state_;
      mutable std::mutex mtx_;

    public:
      QuadStateGuard(QuadState state = QuadState())
        : state_(state){}

      void Write(const QuadState& state);
      void Read(QuadState& state) const;

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline void QuadStateGuard::Write(const QuadState& state) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->state_ = state;
  }

  inline void QuadStateGuard::Read(QuadState& state) const {
    std::lock_guard<std::mutex> lock(this->mtx_);
    state = this->state_;
  }
}
