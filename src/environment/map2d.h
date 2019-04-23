// Author: Tucker Haydon

#pragma once 

#include <vector>
#include <cstdlib>
#include <iostream>

#include "polygon.h"
#include "yaml-cpp/yaml.h"

namespace mediation_layer {
  // The Map3D class encapsulates data about static obstacles and map
  // boundaries. The map maintains data structures, setters, and accessors to
  // the various planes, lines, and vertices that make up a map.
  class Map2D {
    private:
      // The boundary of the map is represented by a convex polygon
      Polygon boundary_;

      // The obstacles in the map are represented by a list of convex polygons
      std::vector<Polygon> obstacles_;

      // Forward-declare friend class for parsing
      friend class YAML::convert<Map2D>;

    public:
      Map2D(const Polygon& boundary = Polygon(),
            const std::vector<Polygon>& obstacles = {})
        : boundary_(boundary),
          obstacles_(obstacles) {}

      // Boundary accessor
      const Polygon& Boundary() const;

      // Boundry setter
      bool SetBoundary(const Polygon& boundary);

      // Obstacles accessor
      const std::vector<Polygon>& Obstacles() const;

      // Obstacles setter
      bool SetObstacles(const std::vector<Polygon>& obtacles);

      // Determines whether or not a point is contained within the map
      bool Contains(const Point2D& point) const;

      // Determines whether or not a point is considered free space. A point is
      // free space if it is contained in the map and not contained in any
      // obstacle
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

namespace YAML {
template<>
struct convert<mediation_layer::Map2D> {
  static Node encode(const mediation_layer::Map2D& rhs) {
    Node node;
    node["boundary"] = rhs.boundary_;
    node["obstacles"] = rhs.obstacles_;
    return node;
  }

  static bool decode(const Node& node, mediation_layer::Map2D& rhs) {
    if(!node.IsMap() || !node["boundary"]) {
      return false;
    }

    rhs.boundary_ = node["boundary"].as<mediation_layer::Polygon>();

    if(node["obstacles"]) {
      rhs.obstacles_ = node["obstacles"].as<std::vector<mediation_layer::Polygon>>();
    }

    return true;
  }
};
}
