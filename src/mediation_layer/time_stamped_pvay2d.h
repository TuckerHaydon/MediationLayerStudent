// Author: Tucker Haydon

#ifndef PATH_PLANNING_MEDIATION_LAYER_TIME_STAMPED_PVAY2D_H
#define PATH_PLANNING_MEDIATION_LAYER_TIME_STAMPED_PVAY2D_H

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
#endif
