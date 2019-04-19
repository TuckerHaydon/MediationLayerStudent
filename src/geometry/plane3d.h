// Author: Tucker Haydon

#pragma once 

#include <vector>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include "plane3d.h"
#include "line3d.h"

namespace mediation_layer {
  // 3D plane object. A 3D plane is built from a set of 3D edges. The edges
  // should be sequential, connected, and the cross product of adjacent edges
  // form a vector that determines the plane's direction. If these assumptions
  // are broken, the behaivor of internal functions is undefined.
  //
  // TODO: Provide checks for assumptions
  class Plane3D {
    private:
      std::vector<Line3D> edges_;
      friend class YAML::convert<Plane3D>;

    public:
      Plane3D(const std::vector<Line3D>& edges = {})
        : edges_(edges) {};

      const std::vector<Line3D>& Edges() const;

      // A point is on the left side of a plane if, given given a
      // counter-clockwise-ordered set of edges, the dot product between the
      // point and the surface normal (the cross product between two
      // counter-clockwise edges) is positive
      bool OnLeftSide(const Point3D& point) const;

      // Returns the standard plane equation
      Eigen::Vector<double, 4> Equation() const;

      // Determines a point on the plane that is closest to the parameter point.
      Point3D ClosestPoint(const Point3D& point) const;

      // Determines if a point is contained in the convex region bounded by the
      // edges
      bool Contains(const Point3D& point) const;

      // Returns the vector normal to the surface of the plane
      Vec3D NormalVector() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Eigen::Vector<double, 4> Plane3D::Equation() const {
    // Contains the equation of the plane. The equation is defined as:
    //   Ax + By + Cz + D = 0
    // The vector is ordered as [A,B,C,D];
    
    // Determine the equation for the plane. The equation can be found in the
    // following manner:
    //  1) Take the cross product of two vectors in the plane: v
    //  2) A = v(0), B = v(1), C = v(2), D = dot(v, -point) where point is any
    //     point on the plane
    Eigen::Vector<double, 4> equation;
    const Vec3D cross_vec = this->edges_[0].AsVector().cross(this->edges_[1].AsVector());
    equation(0) = cross_vec[0];
    equation(1) = cross_vec[1];
    equation(2) = cross_vec[2];
    equation(3) = cross_vec.dot(-1*this->edges_[0].Start());
    return equation;
  }

  inline const std::vector<Line3D>& Plane3D::Edges() const {
    return this->edges_;
  }

  inline bool Plane3D::OnLeftSide(const Point3D& point) const {
    return (point - this->edges_[0].Start()).dot(
        this->edges_[0].AsVector().cross(
          this->edges_[1].AsVector())) > 0;
  }

  inline Point3D Plane3D::ClosestPoint(const Point3D& point) const {
    const Eigen::Vector<double, 4> equation = this->Equation();
    const double A = equation(0);
    const double B = equation(1);
    const double C = equation(2);
    const double D = equation(3);
    const double xp = point(0);
    const double yp = point(1);
    const double zp = point(2);
    const double A2 = std::pow(A,2);
    const double B2 = std::pow(B,2);
    const double C2 = std::pow(C,2);

    // Derive via lagrange multiplier method
    return Point3D (
        -(- xp*B2 + A*yp*B - xp*C2 + A*zp*C + A*D),
        -(- yp*A2 + B*xp*A - yp*C2 + B*zp*C + B*D), 
        -(- zp*A2 + C*xp*A - zp*B2 + C*yp*B + C*D)
        ) / (A2 + B2 + C2);
  }

  inline bool Plane3D::Contains(const Point3D& point) const {
    // Does not satisfy plane equation
    if(std::abs((Eigen::Vector4d(
            point(0), 
            point(1), 
            point(2), 
            1.0)).dot(this->Equation())) > 1e-4) {
      return false;
    }

    const Vec3D normal = this->edges_[0].AsVector().cross(this->edges_[1].AsVector());
    for(const Line3D& edge: this->edges_) {
      if(0 > normal.dot((edge.AsVector()).cross((point - edge.Start())))) {
        return false;
      }
    }
    return true;
  }

  inline Vec3D Plane3D::NormalVector() const {
    return this->edges_[0].AsVector().cross(this->edges_[1].AsVector()).normalized();
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
