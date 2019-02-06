// Author: Tucker Haydon

#ifndef GEOMETRY_POLYHEDRON_H
#define GEOMETRY_POLYHEDRON_H

#include <vector>
#include <iostream>
#include <cstdlib>

#include "plane3d.h"
#include "point3d.h"

namespace path_planning {
  class Polyhedron {
    private:
      std::vector<Plane3D> faces_;

    public: 
      Polyhedron(const std::vector<Plane3D>& faces = {}) 
        : faces_(faces) {}

      const std::vector<Plane3D>& Faces() const;
      bool SetFaces(const std::vector<Plane3D>& faces);

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

  inline bool Polyhedron::SetFaces(const std::vector<Plane3D>& faces) {
    this->faces_ = faces;
    return true;
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

#endif
