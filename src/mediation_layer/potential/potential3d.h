// Author: Tucker Haydon

#pragma once

#include "types.h"

namespace mediation_layer {
  class Potential3D {
    public:
      virtual ~Potential3D() = default;
      virtual Vec3D Resolve(const Point3D& point) const = 0;
  }; 
}
