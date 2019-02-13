// Author: Tucker Haydon

#pragma once

#include "pvay2d.h"

namespace path_planning {
  struct TimeStampedPVAY2D {
    PVAY2D pvay_;
    double time_;
  
    TimeStampedPVAY2D(const PVAY2D& pvay, 
                      const double time)
      : pvay_(pvay),
        time_(time) {}
  };
}
