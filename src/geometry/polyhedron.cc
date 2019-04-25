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
}
