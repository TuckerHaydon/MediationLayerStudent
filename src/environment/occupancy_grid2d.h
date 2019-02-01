// Author: Tucker Haydon

#ifndef PATH_PLANNING_ENVIRONMENT_OCCUPANCY_GRID2D_H
#define PATH_PLANNING_ENVIRONMENT_OCCUPANCY_GRID2D_H

#include <string>
#include <cstdlib>

#include "map2d.h"

namespace path_planning {
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

      bool LoadFromFile(const std::string& file_path);
      bool LoadFromMap(const Map2D& map, const double sample_delta, const double safety_bound=0);
      bool LoadFromBuffer(const bool** buffer, const size_t size_x, const size_t size_y);

      size_t SizeX() const;
      size_t SizeY() const;

      bool IsOccupied(const size_t y, const size_t x) const;

      const bool** Data() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline OccupancyGrid2D::~OccupancyGrid2D() {
    if(false == this->heap_allocated_) { return; }

    // Deallocate memory on heap
    for(size_t idx = 0; idx < this->size_y_; ++idx) {
      std::free(this->data_[idx]);
    }
    std::free(this->data_);
  }

  inline size_t OccupancyGrid2D::SizeX() const {
    return this->size_x_;
  }

  inline size_t OccupancyGrid2D::SizeY() const {
    return this->size_y_;
  }

  inline bool OccupancyGrid2D::IsOccupied(const size_t y, const size_t x) const {
    return this->data_[y][x];
  }

  inline const bool** OccupancyGrid2D::Data() const {
    return const_cast<const bool**>(this->data_);
  }
}

#endif
