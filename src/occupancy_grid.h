// Author: Tucker Haydon

#ifndef PATH_PLANNING_OCCUPANCY_GRID_H
#define PATH_PLANNING_OCCUPANCY_GRID_H

#include "graph.h"

namespace path_planning {
  class OccupancyGrid {
    private:
      bool** occupancy_grid_;
      size_t rows_, cols_;
      bool heap_allocated_{false};

    public:
      OccupancyGrid(bool** occupancy_grid, size_t rows, size_t cols)
      : occupancy_grid_(occupancy_grid),
        rows_(rows),
        cols_(cols)
      {}

      OccupancyGrid(const std::string& file_path);
      ~OccupancyGrid();

      Graph ToGraph();
  };
}

#endif
