// Author: Tucker Haydon

#pragma once

#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

#include "map3d.h"
#include "graph.h"
#include "node_eigen.h"

namespace mediation_layer {
  class OccupancyGrid3D {
    private:
      // Vector that contains the occupancy information
      std::vector<bool> data_;

      double size_x_, size_y_, size_z_;
      double num_cells_x_, num_cells_y_, num_cells_z_;
      double sample_delta_;
      std::vector<std::pair<double, double>> extents_;

      size_t GetFlatIdx(int x_idx, int y_idx, int z_idx) const;
      bool IsValidCoordinate(int x_idx, int y_idx, int z_idx) const;

    public:
      OccupancyGrid3D() {}
      ~OccupancyGrid3D() {}

      // Prevent copies due to heap-allocated resouces
      OccupancyGrid3D(const OccupancyGrid3D&) = delete;
      OccupancyGrid3D& operator=(const OccupancyGrid3D&) = delete;

      // Prevent moves (for now)
      OccupancyGrid3D& operator=(OccupancyGrid3D&& other) noexcept = delete;
      OccupancyGrid3D(OccupancyGrid3D&& other) noexcept  = delete;

      // Load from various entities
      bool LoadFromMap(const Map3D& map, 
                       const double sample_delta, 
                       const double safety_bound=0);

      // Create a graph representation of this occupancy grid.
      Graph3D AsGraph() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline size_t OccupancyGrid3D::GetFlatIdx(int x_idx, int y_idx, int z_idx) const {
    // Original[HEIGHT, WIDTH, DEPTH]
    // Flat[x + WIDTH * (y + DEPTH * z)] = Original[x, y, z]
    return x_idx + this->num_cells_y_ * (y_idx + this->num_cells_z_ * z_idx);
  }

  inline bool OccupancyGrid3D::IsValidCoordinate(int x_idx, int y_idx, int z_idx) const {
    return
      (x_idx >= 0) && 
      (y_idx >= 0) && 
      (z_idx >= 0) && 
      (x_idx < this->num_cells_x_) && 
      (y_idx < this->num_cells_y_) && 
      (z_idx < this->num_cells_z_);
  }

  inline bool OccupancyGrid3D::LoadFromMap(
      const Map3D& map, 
      const double sample_delta, 
      const double safety_bound) {

    // TODO: Implement safety bound

    this->extents_ = map.Extents();
    this->size_x_ = this->extents_[0].second - this->extents_[0].first;
    this->size_y_ = this->extents_[1].second - this->extents_[1].first;
    this->size_z_ = this->extents_[2].second - this->extents_[2].first;

    this->sample_delta_ = sample_delta;

    this->num_cells_x_ = std::ceil(this->size_x_ / this->sample_delta_);
    this->num_cells_y_ = std::ceil(this->size_y_ / this->sample_delta_);
    this->num_cells_z_ = std::ceil(this->size_z_ / this->sample_delta_);

    // Assume all spaces are free
    this->data_ = std::vector<bool>(
        this->num_cells_x_ * this->num_cells_y_ * this->num_cells_z_, 
        0);

    for(size_t z_idx = 0; z_idx < this->num_cells_z_; ++z_idx) {
      for(size_t y_idx = 0; y_idx < this->num_cells_y_; ++y_idx) {
        for(size_t x_idx = 0; x_idx < this->num_cells_x_; ++x_idx) {
          const Point3D p(
            x_idx * this->sample_delta_ + this->extents_[0].first,
            y_idx * this->sample_delta_ + this->extents_[1].first,
            z_idx * this->sample_delta_ + this->extents_[2].first
              );
          if(!map.Contains(p) || !map.IsFreeSpace(p)) {
            const size_t idx = this->GetFlatIdx(x_idx, y_idx, z_idx);
            this->data_[idx] = true;
          }
        }
      }
    }
                 
    return true;
  }

  inline Graph3D OccupancyGrid3D::AsGraph() const {
    // Transform occupancy vector into node vector
    std::vector<std::shared_ptr<Node3D>> node_vector(this->data_.size());
    for(size_t z_idx = 0; z_idx < this->num_cells_z_; ++z_idx) {
      for(size_t y_idx = 0; y_idx < this->num_cells_y_; ++y_idx) {
        for(size_t x_idx = 0; x_idx < this->num_cells_x_; ++x_idx) {
          const Point3D p(
            x_idx * this->sample_delta_ + this->extents_[0].first,
            y_idx * this->sample_delta_ + this->extents_[1].first,
            z_idx * this->sample_delta_ + this->extents_[2].first
              );

          // Original[HEIGHT, WIDTH, DEPTH]
          // Flat[x + WIDTH * (y + DEPTH * z)] = Original[x, y, z]
          const size_t idx = this->GetFlatIdx(x_idx, y_idx, z_idx);
          node_vector[idx] = std::make_shared<Node3D>(p); 
        }
      }
    }

    // Convert node grid to directed edges and add to graph
    std::vector<DirectedEdge3D> edges;
    for(size_t z_idx = 0; z_idx < this->num_cells_z_; ++z_idx) {
      for(size_t y_idx = 0; y_idx < this->num_cells_y_; ++y_idx) {
        for(size_t x_idx = 0; x_idx < this->num_cells_x_; ++x_idx) {
          // Original[HEIGHT, WIDTH, DEPTH]
          // Flat[x + WIDTH * (y + DEPTH * z)] = Original[x, y, z]
          const size_t sink_idx = this->GetFlatIdx(x_idx, y_idx, z_idx);

				  // If current node is unreachable, pass	
				  if(true == this->data_[sink_idx]) { continue; }

          for(int delta_x = -1; delta_x <= 1; ++delta_x) {
            for(int delta_y = -1; delta_y <= 1; ++delta_y) {
              for(int delta_z = -1; delta_z <=1; ++delta_z) {
                if((delta_x == 0) && (delta_y == 0) && (delta_z == 0)) {
                  continue;
                }

                const int x = x_idx + delta_x;
                const int y = y_idx + delta_y;
                const int z = z_idx + delta_z;

                if( true == this->IsValidCoordinate(x,y,z) ) { 
                  const size_t source_idx = this->GetFlatIdx(x,y,z);
                  edges.emplace_back(node_vector[source_idx], node_vector[sink_idx], 
                      std::sqrt(
                        std::pow(delta_x, 2) + 
                        std::pow(delta_y, 2) + 
                        std::pow(delta_z, 2))); 
                }
              }
            }
          }
        }
      }
    }

    return Graph3D(edges);
  }
}
