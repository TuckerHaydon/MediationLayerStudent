// Author: Tucker Haydon

#pragma once

#include <mutex>

#include "quad_state.h"

namespace game_engine {
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
}
