// Author: Tucker Haydon

#ifndef PATH_PLANNING_MEDIATION_LAYER_TRAJECTORY2D_H
#define PATH_PLANNING_MEDIATION_LAYER_TRAJECTORY2D_H

#include <vector>

#include "time_stamped_pvay2d.h"

namespace path_planning {
  struct Trajectory2D {
    std::vector<TimeStampedPVAY2D> data_;

    Trajectory2D(const std::vector<TimeStampedPVAY2D>& data = {}) 
      : data_(data) {}
  };
}

#endif
