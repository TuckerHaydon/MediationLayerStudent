// Author: Tucker Haydon

#ifndef GEOMETRY_PLANE3D_H
#define GEOMETRY_PLANE3D_H

#include <vector>

namespace path_planning {
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
    return (point - this->edges_[0].Start()).Dot(
        this->edges_[0].AsVector().Cross(
          this->edges_[1].AsVector())) > 0;
  }
}
#endif
