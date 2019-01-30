// Author: Tucker Haydon

#ifndef PATH_PLANNING_ENVIRONMENT_MAP2D_H
#define PATH_PLANNING_ENVIRONMENT_MAP2D_H

#include <vector>
#include <cstdlib>
#include <iostream>

#include "polygon.h"
#include "point2d.h"

namespace path_planning {
  class Map2D {
    private:
      geometry::Polygon boundary_;
      std::vector<geometry::Polygon> obstacles_;

    public:
      Map2D() {}

      const geometry::Polygon& Boundary() const;
      bool SetBoundary(const geometry::Polygon& boundary);

      const std::vector<geometry::Polygon>& Obstacles() const;
      bool SetObstacles(const std::vector<geometry::Polygon>& obtacles);

      bool Contains(const geometry::Point2D& point) const;
      bool IsFreeSpace(const geometry::Point2D& point) const;

      Map2D Inflate(const double distance) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const geometry::Polygon& Map2D::Boundary() const {
    return this->boundary_;
  } 

  inline bool Map2D::SetBoundary(const geometry::Polygon& boundary) {
    this->boundary_ = boundary;
    return true;
  }

  inline const std::vector<geometry::Polygon>& Map2D::Obstacles() const {
    return this->obstacles_;
  }

  inline bool Map2D::SetObstacles(const std::vector<geometry::Polygon>& obstacles) {
    this->obstacles_ = obstacles;
    return true;
  }

  inline bool Map2D::Contains(const geometry::Point2D& point) const {
    return this->boundary_.Contains(point);
  }

  inline bool Map2D::IsFreeSpace(const geometry::Point2D& point) const {
    for(const geometry::Polygon& obstacle: this->obstacles_)  {
      if(true == obstacle.Contains(point)) {
        return false;
      }
    }
    return true;
  }

  inline Map2D Map2D::Inflate(const double distance) const {
    std::cerr << "Map2D::Inflate(double) is not yet implemented!" << std::endl; 
    std::exit(EXIT_FAILURE);

    return Map2D();
  }

}
#endif
