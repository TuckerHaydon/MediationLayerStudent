// Author: Tucker Haydon

#pragma once

#include <vector>

#include "time_stamped_pvay2d.h"

namespace path_planning {
  struct Trajectory2D {
    std::vector<TimeStampedPVAY2D> data_;

    Trajectory2D(const std::vector<TimeStampedPVAY2D>& data = {}) 
      : data_(data) {}
  };
}
