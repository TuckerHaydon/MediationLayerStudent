// Author: Tucker Haydon

#pragma once

#include <algorithm>
#include <cassert>

namespace mediation_layer {
  /*
   * POD structure containing time span information used during integrator
   */
  struct TimeSpan {
    const double t0_;
    const double tf_;
    const double dt_;

    TimeSpan(const double t0,
             const double tf,
             const double dt = 0.0) 
      : t0_(t0),
        tf_(tf),
        dt_(std::max(tf - t0, dt)) {
      assert(tf_ > t0_);
      assert(dt_ > 0.0);
      assert(dt_ <= tf_ - t0_);
    }
  };
}
