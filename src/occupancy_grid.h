// Author: Tucker Haydon

#ifndef PATH_PLANNING_OCCUPANCY_GRID_H
#define PATH_PLANNING_OCCUPANCY_GRID_H

#include <cstddef>
#include <string>
#include <utility>

namespace path_planning {

  // Forward Declare
  class Graph;

  class OccupancyGrid {
    private:
      bool** occupancy_grid_;
      size_t rows_, cols_;
      bool heap_allocated_{false};
      friend class Graph;

    public:
      OccupancyGrid(const std::string& file_path);
      ~OccupancyGrid();

      // Prevent copies due to heap-allocated resouces
      OccupancyGrid(const OccupancyGrid&) = delete;
      OccupancyGrid& operator=(OccupancyGrid const&) = delete;

      // Prevent moves (for now)
      OccupancyGrid& operator=(OccupancyGrid&& other) noexcept = delete;
      OccupancyGrid(OccupancyGrid&& other) noexcept  = delete;
  };
}

#endif
