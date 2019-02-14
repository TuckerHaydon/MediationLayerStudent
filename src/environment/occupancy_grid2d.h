// Author: Tucker Haydon

#pragma once

#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

#include "map2d.h"
#include "graph.h"
#include "node2d.h"

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

  inline bool OccupancyGrid2D::LoadFromFile(const std::string& file_path) {
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

  inline bool OccupancyGrid2D::LoadFromMap(
      const Map2D& map, 
      const double sample_delta, 
      const double safety_bound) {
    double min_x{std::numeric_limits<double>::max()},
           min_y{std::numeric_limits<double>::max()},
           max_x{std::numeric_limits<double>::min()},
           max_y{std::numeric_limits<double>::min()};

    for(const Point2D& vertex: map.Boundary().Vertices()) {
      if(vertex.x() < min_x) { min_x = vertex.x(); }
      if(vertex.y() < min_y) { min_y = vertex.y(); }
      if(vertex.x() > max_x) { max_x = vertex.x(); }
      if(vertex.y() > max_y) { max_y = vertex.y(); }
    }

    this->size_y_ = std::ceil((max_y - min_y) / sample_delta) + 1;
    this->size_x_= std::ceil((max_x - min_x) / sample_delta) + 1;

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
        const Point2D p(min_x + col*sample_delta, min_y+row*sample_delta);
        // True indicates occupied, false indicates free
        this->data_[row][col] = !inflated_map.Contains(p) || !inflated_map.IsFreeSpace(p);
      }
    }
                 
    return false;
  }

  inline bool OccupancyGrid2D::LoadFromBuffer(
      const bool** buffer,
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

  inline Graph2D OccupancyGrid2D::AsGraph() const {
    // Build a 2D array of nodes
    std::shared_ptr<Node2D> node_grid[this->size_y_][this->size_x_];
    for(size_t row = 0; row < this->size_y_; ++row) {
      for(size_t col = 0; col < this->size_x_; ++col) {
        node_grid[row][col] = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(row, col));
      }
    }

    // Convert node grid to directed edges and add to graph
    std::vector<DirectedEdge2D> edges;
    for(int row = 0; row < this->size_y_; ++row) {
      for(int col = 0; col < this->size_x_; ++col) {
				// If current node is unreachable, pass	
				if(true == this->IsOccupied(row, col)) { continue; }

        constexpr double ADJACENT_COST = 1.0;
        constexpr double DIAGONAL_COST = std::sqrt(2);
				// Else, create paths from nearby nodes into this one
        // 8 node surrounding the current node
        if(row - 1 >= 0) { edges.emplace_back(node_grid[row - 1][col], node_grid[row][col], ADJACENT_COST); }
        if(col - 1 >= 0) { edges.emplace_back(node_grid[row][col - 1], node_grid[row][col], ADJACENT_COST); }

        if(row + 1 < this->size_y_) { edges.emplace_back(node_grid[row + 1][col], node_grid[row][col], ADJACENT_COST); }
        if(col + 1 < this->size_x_) { edges.emplace_back(node_grid[row][col + 1], node_grid[row][col], ADJACENT_COST); }

        if(row - 1 >= 0 && col - 1 >= 0) { 
          edges.emplace_back(node_grid[row - 1][col - 1], node_grid[row][col], DIAGONAL_COST); }
        if(row - 1 >= 0 && col + 1 < this->size_x_) { 
          edges.emplace_back(node_grid[row - 1][col + 1], node_grid[row][col], DIAGONAL_COST); }
        if(row + 1 < this->size_y_ && col - 1 >= 0) { 
          edges.emplace_back(node_grid[row + 1][col - 1], node_grid[row][col], DIAGONAL_COST); }
        if(row + 1 < this->size_y_ && col +1 < this->size_x_) { 
          edges.emplace_back(node_grid[row + 1][col + 1], node_grid[row][col], DIAGONAL_COST); }
      }
    }

    return Graph2D(edges);
  }
}
