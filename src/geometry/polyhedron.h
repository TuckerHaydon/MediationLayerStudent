// Author: Tucker Haydon

#pragma once

#include <vector>
#include <iostream>
#include <cstdlib>

#include "yaml-cpp/yaml.h"
#include "plane3d.h"

namespace game_engine {
  // A polyhedron is represented by a connected set of convex polygons that
  // encapsulate a closed, convex, 3D space.
  //
  // TODO: Put checks into place to ensure that the polyhedron is convex and
  // closed
  class Polyhedron {
    private:
      // Set of convex polygons embedded in a 3D space that make up the boundary
      // of the polyhedron
      std::vector<Plane3D> faces_;

      // Forward-declare parser
      friend class YAML::convert<Polyhedron>;

    public: 
      // Constructor
      Polyhedron(const std::vector<Plane3D>& faces = {}) 
        : faces_(faces) {}

      // Faces accessor
      const std::vector<Plane3D>& Faces() const;

      // Determines if a 3D point is contained within the polyhedron. A point is
      // contained within a polyhedron if, given a convex polyhedron and a set
      // of faces whose normal vectors are pointing inwards, the point is always
      // to the left of the faces
      bool Contains(const Point3D& point) const;

      // Determines if the polyhedron is convex
      bool IsConvex() const;

      // Returns the convex hull of the polyhedron
      Polyhedron ConvexHull() const;
  };
}

namespace YAML {
  template<>
  struct convert<game_engine::Polyhedron> {
    static Node encode(const game_engine::Polyhedron& rhs) {
      Node node;
      node.push_back(rhs.faces_);
      return node;
    }
  
    static bool decode(const Node& node, game_engine::Polyhedron& rhs) {
      if(!node.IsSequence()) {
        return false;
      }
  
      std::vector<game_engine::Plane3D> faces;
      faces.reserve(node.size());
      for(size_t idx = 0; idx < node.size(); ++idx) {
        faces.push_back(node[idx].as<game_engine::Plane3D>());
      }
  
      rhs.faces_ = faces;
      return true;
    }
  };
}
