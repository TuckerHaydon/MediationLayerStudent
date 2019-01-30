// Author: Tucker Haydon

#ifndef PATH_PLANNING_GUI_GNU_VISUALIZER
#define PATH_PLANNING_GUI_GNU_VISUALIZER

#include <vector>

#include "node.h"
#include "occupancy_grid2d.h"

namespace path_planning { 
  struct GNUVisualizer {
  
    static void Run(const OccupancyGrid2D& occupancy_grid, 
                    const std::vector<Node>& path);
  };
}

#endif
