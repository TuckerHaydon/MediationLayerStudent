// Author: Tucker Haydon

#pragma once 

#include <vector>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <utility>

#include "polyhedron.h"
#include "yaml-cpp/yaml.h"

namespace mediation_layer {
  // The Map3D class encapsulates data about static obstacles and map
  // boundaries. The map maintains data structures, setters, and accessors to
  // the various planes, lines, and vertices that make up a map.
  class Map3D {
    private:
      // The boundary of the map is represented by a convex polyhedron
      Polyhedron boundary_;

      // The obstacles in the map are represented by a list of convex polyhedra
      std::vector<Polyhedron> obstacles_;

      // Forward-declare friend class for parsing
      friend class YAML::convert<Map3D>;

    public:
      // Constructor
      Map3D(const Polyhedron& boundary = Polyhedron(),
            const std::vector<Polyhedron>& obstacles = {})
        : boundary_(boundary),
          obstacles_(obstacles) {}

      // Boundary accessor
      const Polyhedron& Boundary() const;

      // Obstacles accessor
      const std::vector<Polyhedron>& Obstacles() const;

      // Determines whether or not a point is contained within the map
      bool Contains(const Point3D& point) const;

      // Determines whether or not a point is considered free space. A point is
      // free space if it is contained in the map and not contained in any
      // obstacle
      bool IsFreeSpace(const Point3D& point) const;

      // Determines the extents of the map. Returns a list of tuples that
      // contain the {min,max} coordinates for the XYZ dimensions.
      std::vector<std::pair<double, double>> Extents() const;

      // Returns the plane with the smallest average z-coordinate
      Plane3D Ground() const;

      // Returns the list of planes minus the ground plane
      std::vector<Plane3D> Walls() const;

      // Inflates a map by a set distance. Map boundaries are shrunk and
      // obstacles are expanded. Shrinking and expanding affects both x and y
      // directions equally, therefore object aspect ratios are not guaranteed
      // to stay the same.
      Map3D Inflate(const double distance) const;
  };
}

namespace YAML {
template<>
struct convert<mediation_layer::Map3D> {
  static Node encode(const mediation_layer::Map3D& rhs) {
    Node node;
    node["boundary"] = rhs.boundary_;
    node["obstacles"] = rhs.obstacles_;
    return node;
  }

  static bool decode(const Node& node, mediation_layer::Map3D& rhs) {
    if(!node.IsMap() || !node["boundary"]) {
      return false;
    }

    rhs.boundary_ = node["boundary"].as<mediation_layer::Polyhedron>();

    if(node["obstacles"]) {
      rhs.obstacles_ = node["obstacles"].as<std::vector<mediation_layer::Polyhedron>>();
    }

    return true;
  }
};
}
