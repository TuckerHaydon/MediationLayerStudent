// Author: Tucker Haydon

#ifndef PATH_PLANNING_GNU_VISUALIZER
#define PATH_PLANNING_GNU_VISUALIZER

#include <vector>

#include "node.h"
#include "occupancy_grid.h"

namespace path_planning { 
  struct GNUVisualizer {
  
    static void Run(const OccupancyGrid& occupancy_grid, 
                    const std::vector<Node>& path);
  };
}

#endif
