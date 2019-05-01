// Author: Tucker Haydon

#include "polyhedron.h"

namespace game_engine {
  const std::vector<Plane3D>& Polyhedron::Faces() const {
    return this->faces_;
  }

  bool Polyhedron::Contains(const Point3D& point) const {
    for(const Plane3D& face: this->faces_) {
      if(false == face.OnLeftSide(point)) { return false; }
    }

    return true;
  }

  bool Polyhedron::IsConvex() const {
    std::cerr << "Polyhedron::IsConvex() is not yet implemented!" << std::endl; 
    std::exit(EXIT_FAILURE);

    return false;
  }

  Polyhedron Polyhedron::ConvexHull() const {
    std::cerr << "Polyhedron::ConvexHull() is not yet implemented!" << std::endl; 
    std::exit(EXIT_FAILURE);

    return Polyhedron();
  }

  Point3D Polyhedron::InteriorPoint() const {
    Point3D sum(0,0,0);
    size_t counter = 0;
    for(const Plane3D& face: this->Faces()) {
      for(const Line3D edge: face.Edges()) {
        sum = sum + edge.Start() + edge.End();
        counter+=2;
      }
    }
    return sum / counter;
  }

  Polyhedron Polyhedron::Shrink(const double distance) const {
    return this->Expand(-1*distance);
  }

  Polyhedron Polyhedron::Expand(const double distance) const {
    const Point3D interior_point = this->InteriorPoint();
    std::vector<Plane3D> new_faces;
    for(const Plane3D& old_face: this->faces_) {
      std::vector<Line3D> new_edges;
      for(const Line3D& old_edge: old_face.Edges()) {
        const Vec3D start_vec = old_edge.Start() - interior_point;
        const Vec3D end_vec = old_edge.End() - interior_point;

        auto sign = [](double el) {
          if(el < 0) {
            return -1;
          } if(el > 0) {
            return +1;
          } else {
            return 0;
          }
        };

        const Vec3D unit_start = start_vec.unaryExpr(sign).cast<double>();
        const Vec3D unit_end = end_vec.unaryExpr(sign).cast<double>();

        const Point3D new_start = old_edge.Start() + distance * unit_start; 
        const Point3D new_end = old_edge.End() + distance * unit_end; 

        new_edges.emplace_back(new_start, new_end);
      }
      new_faces.emplace_back(new_edges);
    }
    return Polyhedron(new_faces);
  }
}
