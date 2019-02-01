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

  bool OccupancyGrid2D::LoadFromMap(const Map2D& map, 
                                    const double sample_delta, 
                                    const double safety_bound) {
    double min_x{std::numeric_limits<double>::max()},
           min_y{std::numeric_limits<double>::max()},
           max_x{std::numeric_limits<double>::min()},
           max_y{std::numeric_limits<double>::min()};

    for(geometry::Point2D vertex: map.Boundary().Vertices()) {
      if(vertex.X() < min_x) { min_x = vertex.X(); }
      if(vertex.Y() < min_y) { min_y = vertex.Y(); }
      if(vertex.X() > max_x) { max_x = vertex.X(); }
      if(vertex.Y() > max_y) { max_y = vertex.Y(); }
    }

    this->size_y_ = std::ceil((max_y - min_y) / sample_delta);
    this->size_x_= std::ceil((max_x - min_x) / sample_delta);

    // Allocate memory on the heap for the file
    this->data_ = 
      reinterpret_cast<bool**>(std::malloc(this->size_y_ * sizeof(bool*)));
    for(size_t idx = 0; idx < this->size_y_; ++idx) {
      this->data_[idx] = 
        reinterpret_cast<bool*>(std::malloc(this->size_x_ * sizeof(bool)));
    }
    this->heap_allocated_ = true;
    
    const Map2D inflated_map = map.Inflate(safety_bound);

    // Read in file
    for(size_t row = 0; row < this->size_y_; ++row) {
      for(size_t col = 0; col < this->size_x_; ++col) {
        const geometry::Point2D p(min_x + col*sample_delta, min_y+row*sample_delta);
        // True indicates occupied, false indicates free
        this->data_[row][col] = !inflated_map.Contains(p) || !inflated_map.IsFreeSpace(p);
      }
    }
                 
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
