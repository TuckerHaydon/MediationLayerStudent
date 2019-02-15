// Author: Tucker Haydon

#pragma once 

#include <vector>

#include "line3d.h"

namespace mediation_layer {
  class Plane3D {
    private:
      std::vector<Line3D> edges_;

    public:
      Plane3D(const std::vector<Line3D>& edges = {})
        : edges_(edges) {}

      const std::vector<Line3D>& Edges() const;
      bool SetEdges(const std::vector<Line3D>& edges);

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

  inline bool Plane3D::SetEdges(const std::vector<Line3D>& edges) {
    this->edges_ = edges;
    return true;
  }

  inline bool Plane3D::OnLeftSide(const Point3D& point) const {
    return (point - this->edges_[0].Start()).dot(
        this->edges_[0].AsVector().cross(
          this->edges_[1].AsVector())) > 0;
  }
}
