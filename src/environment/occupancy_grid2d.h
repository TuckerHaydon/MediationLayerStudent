// Author: Tucker Haydon

#pragma once

#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

#include "map2d.h"
#include "graph.h"
#include "node_eigen.h"

namespace game_engine {
  class OccupancyGrid2D {
    private:
      bool** data_;
      size_t size_x_, size_y_;
      bool heap_allocated_{false};

    public:
      OccupancyGrid2D() {}
      ~OccupancyGrid2D();

      // Prevent copies due to heap-allocated resouces
      OccupancyGrid2D(const OccupancyGrid2D&) = delete;
      OccupancyGrid2D& operator=(const OccupancyGrid2D&) = delete;

      // Prevent moves (for now)
      OccupancyGrid2D& operator=(OccupancyGrid2D&& other) noexcept = delete;
      OccupancyGrid2D(OccupancyGrid2D&& other) noexcept  = delete;

      // Load from various entities
      bool LoadFromFile(const std::string& file_path);
      bool LoadFromMap(const Map2D& map, const double sample_delta, const double safety_bound=0);
      bool LoadFromBuffer(const bool** buffer, const size_t size_x, const size_t size_y);

      // Create a graph representation of this occupancy grid. Every cell has a
      // directed edge to the 8 cells around it.
      Graph2D AsGraph() const;

      size_t SizeX() const;
      size_t SizeY() const;

      bool IsOccupied(const size_t y, const size_t x) const;

      const bool** Data() const;
  };
}
