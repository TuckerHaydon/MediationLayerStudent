// Author: Tucker Haydon

#include "occupancy_grid2d.h"

#include <cstdlib>
#include <fstream>
#include <iostream>

namespace path_planning {
  bool OccupancyGrid2D::LoadFromFile(const std::string& file_path) {
    std::ifstream f(file_path);
    if(!f.is_open()) {
      std::cerr 
        << "OccupancyGrid2D::LoadFromFile: File could not be opened." 
        << std::endl;
      return false;
    }

    f >> this->size_y_;
    f >> this->size_x_;

    // Allocate memory on the heap for the file
    this->data_ = 
      reinterpret_cast<bool**>(std::malloc(this->size_y_ * sizeof(bool*)));
    for(size_t idx = 0; idx < this->size_y_; ++idx) {
      this->data_[idx] = 
        reinterpret_cast<bool*>(std::malloc(this->size_x_ * sizeof(bool)));
    }
    this->heap_allocated_ = true;

    // Read in file
    for(size_t row = 0; row < this->size_y_; ++row) {
      for(size_t col = 0; col < this->size_x_; ++col) {
        f >> this->data_[row][col];
      }
    }

    f.close();
    return true;
  }

  bool OccupancyGrid2D::LoadFromMap(const Map2D& map, const double delta) {
    std::cerr << "OccupancyGrid2D::LoadFromMap() is not yet implemented!" << std::endl; 
    std::exit(EXIT_FAILURE);

    return false;
  }

  bool OccupancyGrid2D::LoadFromBuffer(const bool** buffer,
                                       const size_t size_x, 
                                       const size_t size_y) {
    this->size_y_ = size_y;
    this->size_x_ = size_x;

    // Allocate memory on the heap for the file
    this->data_ = 
      reinterpret_cast<bool**>(std::malloc(this->size_y_ * sizeof(bool*)));
    for(size_t idx = 0; idx < this->size_y_; ++idx) {
      this->data_[idx] = 
        reinterpret_cast<bool*>(std::malloc(this->size_x_ * sizeof(bool)));
      std::memcpy(&this->data_[idx], &buffer[idx], this->size_x_ * sizeof(bool));
    }
    this->heap_allocated_ = true;

    return true;
  }
}
