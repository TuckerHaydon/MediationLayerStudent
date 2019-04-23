// Author: Tucker Haydon

#pragma once

#include <algorithm>
#include <cassert>

namespace mediation_layer {
  // Data structure containing time span information for use integration
  //
  // TODO: Add checks for valid data
  struct TimeSpan {
    // Start time
    double t0_;

    // End time
    double tf_;

    // Time step
    double dt_;

    // Constructor
    TimeSpan(const double t0,
             const double tf,
             const double dt = 0.0) 
      : t0_(t0),
        tf_(tf),
        dt_(std::max(tf - t0, dt)) {}
  };
}
