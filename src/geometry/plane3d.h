// Author: Tucker Haydon

#pragma once 

#include <vector>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include "plane3d.h"
#include "line3d.h"

namespace mediation_layer {
  // Implementation of a 2D closed, convex polygon embedded in a 3D space. A
  // plane may be represented by a set of continuous 3D lines all lying on the
  // same plane and encapsulating a convex region. 
  //
  // Edges must be specified with the following properties:
  //  1) The end point of each edge must coincide with the start-point of
  //     another edge
  //  2) Edges must be ordered such that the cross product between sequential
  //     edges points towards the center of the convex region
  //  3) All edges must lie on the same 3D plane
  //
  // TODO: Add checks for convex and closed properties
  class Plane3D {
    private:
      // Edges defining the boundaries of the plane
      std::vector<Line3D> edges_;

      // Forward declare parser
      friend class YAML::convert<Plane3D>;

    public:
      // Constructor
      Plane3D(const std::vector<Line3D>& edges = {})
        : edges_(edges) {};

      // Edges accessor
      const std::vector<Line3D>& Edges() const;

      // A point is on the left side of a plane if, given given a
      // counter-clockwise-ordered set of edges, the dot product between the
      // point and the surface normal (the cross product between two
      // counter-clockwise edges) is positive
      bool OnLeftSide(const Point3D& point) const;

      // Return the equation of the plane. The equation is defined as:
      //   Ax + By + Cz + D = 0
      // The vector is ordered as [A,B,C,D];
      Eigen::Vector4d Equation() const;

      // Determines a point on the plane that is closest to the parameter point.
      Point3D ClosestPoint(const Point3D& point) const;

      // Determines if a point is contained in the convex region bounded by the
      // edges
      bool Contains(const Point3D& point) const;

      // Returns the vector normal to the surface of the plane
      Vec3D NormalVector() const;
  };
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
