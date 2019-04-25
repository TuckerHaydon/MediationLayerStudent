// Author: Tucker Haydon

#include <iostream>
#include <cstdlib>

#include "map2d.h"

namespace game_engine {
  const Polygon& Map2D::Boundary() const {
    return this->boundary_;
  } 

  bool Map2D::SetBoundary(const Polygon& boundary) {
    this->boundary_ = boundary;
    return true;
  }

  const std::vector<Polygon>& Map2D::Obstacles() const {
    return this->obstacles_;
  }

  bool Map2D::SetObstacles(const std::vector<Polygon>& obstacles) {
    this->obstacles_ = obstacles;
    return true;
  }

  bool Map2D::Contains(const Point2D& point) const {
    return this->boundary_.Contains(point);
  }

  bool Map2D::IsFreeSpace(const Point2D& point) const {
    for(const Polygon& obstacle: this->obstacles_)  {
      if(true == obstacle.Contains(point)) {
        return false;
      }
    }
    return true;
  }

  Map2D Map2D::Inflate(const double distance) const {
    const Polygon new_boundary = this->boundary_.Shrink(distance);
    std::vector<Polygon> new_obstacles;
    new_obstacles.reserve(this->obstacles_.size());

    for(const Polygon& obstacle: this->obstacles_) {
      new_obstacles.push_back(obstacle.Expand(distance));
    }

    return Map2D(new_boundary, new_obstacles);
  }

  Polygon Map2D::Extents() const {
    return this->boundary_.BoundingBox();
  }
}
