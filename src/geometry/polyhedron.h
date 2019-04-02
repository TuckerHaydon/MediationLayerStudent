// Author: Tucker Haydon

#pragma once

#include <vector>
#include <iostream>
#include <cstdlib>

#include "yaml-cpp/yaml.h"
#include "plane3d.h"

namespace mediation_layer {
  class Polyhedron {
    private:
      std::vector<Plane3D> faces_;
      friend class YAML::convert<Polyhedron>;

    public: 
      Polyhedron(const std::vector<Plane3D>& faces = {}) 
        : faces_(faces) {}

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

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const std::vector<Plane3D>& Polyhedron::Faces() const {
    return this->faces_;
  }

  inline bool Polyhedron::Contains(const Point3D& point) const {
    // TODO: Check polyhedron convex
    // TODO: Check polyhedron complete

    for(const Plane3D& face: this->faces_) {
      if(false == face.OnLeftSide(point)) { return false; }
    }

    return true;
  }

  inline bool Polyhedron::IsConvex() const {
    std::cerr << "Polyhedron::IsConvex() is not yet implemented!" << std::endl; 
    std::exit(EXIT_FAILURE);

    return false;
  }

  inline Polyhedron Polyhedron::ConvexHull() const {
    std::cerr << "Polyhedron::ConvexHull() is not yet implemented!" << std::endl; 
    std::exit(EXIT_FAILURE);

    return Polyhedron();
  }
}

namespace YAML {
  template<>
  struct convert<mediation_layer::Polyhedron> {
    static Node encode(const mediation_layer::Polyhedron& rhs) {
      Node node;
      node.push_back(rhs.faces_);
      return node;
    }
  
    static bool decode(const Node& node, mediation_layer::Polyhedron& rhs) {
      if(!node.IsSequence()) {
        return false;
      }
  
      std::vector<mediation_layer::Plane3D> faces;
      faces.reserve(node.size());
      for(size_t idx = 0; idx < node.size(); ++idx) {
        faces.push_back(node[idx].as<mediation_layer::Plane3D>());
      }
  
      rhs.faces_ = faces;
      return true;
    }
  };
}
