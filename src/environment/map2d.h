// Author: Tucker Haydon

#pragma once 

#include <vector>
#include <cstdlib>
#include <iostream>

#include "polygon.h"

namespace path_planning {
  class Map2D {
    private:
      Polygon boundary_;
      std::vector<Polygon> obstacles_;

    public:
      Map2D(const Polygon& boundary = Polygon(),
            const std::vector<Polygon>& obstacles = {})
        : boundary_(boundary),
          obstacles_(obstacles) {}

      const Polygon& Boundary() const;
      bool SetBoundary(const Polygon& boundary);

      const std::vector<Polygon>& Obstacles() const;
      bool SetObstacles(const std::vector<Polygon>& obtacles);

      bool Contains(const Point2D& point) const;
      bool IsFreeSpace(const Point2D& point) const;

      Polygon Extents() const;

      // Inflates a map by a set distance. Map boundaries are shrunk and
      // obstacles are expanded. Shrinking and expanding affects both x and y
      // directions equally, therefore object aspect ratios are not guaranteed
      // to stay the same.
      Map2D Inflate(const double distance) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const Polygon& Map2D::Boundary() const {
    return this->boundary_;
  } 

  inline bool Map2D::SetBoundary(const Polygon& boundary) {
    this->boundary_ = boundary;
    return true;
  }

  inline const std::vector<Polygon>& Map2D::Obstacles() const {
    return this->obstacles_;
  }

  inline bool Map2D::SetObstacles(const std::vector<Polygon>& obstacles) {
    this->obstacles_ = obstacles;
    return true;
  }

  inline bool Map2D::Contains(const Point2D& point) const {
    return this->boundary_.Contains(point);
  }

  inline bool Map2D::IsFreeSpace(const Point2D& point) const {
    for(const Polygon& obstacle: this->obstacles_)  {
      if(true == obstacle.Contains(point)) {
        return false;
      }
    }
    return true;
  }

  inline Map2D Map2D::Inflate(const double distance) const {
    const Polygon new_boundary = this->boundary_.Shrink(distance);
    std::vector<Polygon> new_obstacles;
    new_obstacles.reserve(this->obstacles_.size());

    for(const Polygon& obstacle: this->obstacles_) {
      new_obstacles.push_back(obstacle.Expand(distance));
    }

    return Map2D(new_boundary, new_obstacles);
  }

  inline Polygon Map2D::Extents() const {
    return this->boundary_.BoundingBox();
  }
}
