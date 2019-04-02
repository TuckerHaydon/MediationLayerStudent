// Author: Tucker Haydon

#pragma once 

#include <vector>

#include "yaml-cpp/yaml.h"
#include "line3d.h"

namespace mediation_layer {
  class Plane3D {
    private:
      std::vector<Line3D> edges_;
      friend class YAML::convert<Plane3D>;

    public:
      Plane3D(const std::vector<Line3D>& edges = {})
        : edges_(edges) {}

      const std::vector<Line3D>& Edges() const;

      // A point is on the left side of a plane if, given given a
      // counter-clockwise-ordered set of edges, the dot product between the
      // point and the surface normal (the cross product between two
      // counter-clockwise edges) is positive
      bool OnLeftSide(const Point3D& point) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const std::vector<Line3D>& Plane3D::Edges() const {
    return this->edges_;
  }

  inline bool Plane3D::OnLeftSide(const Point3D& point) const {
    return (point - this->edges_[0].Start()).dot(
        this->edges_[0].AsVector().cross(
          this->edges_[1].AsVector())) > 0;
  }
}

namespace YAML {
  template<>
  struct convert<mediation_layer::Plane3D> {
    static Node encode(const mediation_layer::Plane3D& rhs) {
      Node node;
      node.push_back(rhs.edges_);
      return node;
    }
  
    static bool decode(const Node& node, mediation_layer::Plane3D& rhs) {
      if(!node.IsSequence()) {
        return false;
      }
  
      std::vector<mediation_layer::Line3D> edges;
      edges.reserve(node.size());
      for(size_t idx = 0; idx < node.size(); ++idx) {
        edges.push_back(node[idx].as<mediation_layer::Line3D>());
      }
  
      rhs.edges_ = edges;
      return true;
    }
  };
}
