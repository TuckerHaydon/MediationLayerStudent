// Author: Tucker Haydon

#ifndef PATH_PLANNING_MEDIATION_LAYER_POTENTIAL2D_H
#define PATH_PLANNING_MEDIATION_LAYER_POTENTIAL2D_H

#include "point2d.h"
#include "vec2d.h"

namespace path_planning {
  class Potential {
    public:
      virtual ~Potential() = default;
      virtual Vec2D Resolve(const Point2D& point) const = 0;
  }; 
}

#endif
