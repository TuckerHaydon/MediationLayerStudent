// Author: Tucker Haydon

#pragma once

#include "types.h"

namespace mediation_layer {
  class Potential2D {
    public:
      virtual ~Potential2D() = default;
      virtual Vec2D Resolve(const Point2D& point) const = 0;
  }; 
}
