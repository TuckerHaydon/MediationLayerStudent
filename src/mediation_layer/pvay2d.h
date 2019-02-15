// Author: Tucker Haydon

#pragma once

#include "types.h"

namespace mediation_layer {
  struct PVAY2D {
    Vec2D position_;
    Vec2D velocity_;
    Vec2D acceleration_;
    double yaw_;

    PVAY2D(const Vec2D& position,
           const Vec2D& velocity,
           const Vec2D& acceleration,
           const double yaw)
      : position_(position),
        velocity_(velocity),
        acceleration_(acceleration),
        yaw_(yaw) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  };
}
