// Author: Tucker Haydon

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "quad_state.h"

namespace mediation_layer {
  // State guard is a wrapper around a QuadState that ensures thread-safe,
  // sychronized access to the QuadState. 
  template <size_t T>
  class QuadStateGuard {
    private:
      QuadState<T> state_;
      mutable std::mutex mtx_;

    public:
      QuadStateGuard(QuadState<T> state = QuadState<T>())
        : state_(state){}

      void Write(const QuadState<T>& state);
      void Read(QuadState<T>& state) const;

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline void QuadStateGuard<T>::Write(const QuadState<T>& state) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->state_ = state;
  }

  template <size_t T>
  inline void QuadStateGuard<T>::Read(QuadState<T>& state) const {
    std::lock_guard<std::mutex> lock(this->mtx_);
    state = this->state_;
  }

  using QuadStateGuard2D = QuadStateGuard<2>;
  using QuadStateGuard3D = QuadStateGuard<3>;
}
