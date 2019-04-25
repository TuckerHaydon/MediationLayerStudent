// Author: Tucker Haydon

#include "plane3d.h"

namespace mediation_layer {
  Eigen::Vector4d Plane3D::Equation() const {
    // Determine the equation for the plane. The equation can be found in the
    // following manner:
    //  1) Take the cross product of two vectors in the plane: v
    //  2) A = v(0), B = v(1), C = v(2), D = dot(v, -point) where point is any
    //     point on the plane
    Eigen::Vector4d equation;
    const Vec3D cross_vec = this->edges_[0].AsVector().cross(this->edges_[1].AsVector());
    equation(0) = cross_vec[0];
    equation(1) = cross_vec[1];
    equation(2) = cross_vec[2];
    equation(3) = cross_vec.dot(-1*this->edges_[0].Start());
    return equation;
  }

  const std::vector<Line3D>& Plane3D::Edges() const {
    return this->edges_;
  }

  bool Plane3D::OnLeftSide(const Point3D& point) const {
    return (point - this->edges_[0].Start()).dot(
        this->edges_[0].AsVector().cross(
          this->edges_[1].AsVector())) > 0;
  }

  Point3D Plane3D::ClosestPoint(const Point3D& point) const {
    const Eigen::Vector4d equation = this->Equation();
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

  bool Plane3D::Contains(const Point3D& point) const {
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

  Vec3D Plane3D::NormalVector() const {
    return this->edges_[0].AsVector().cross(this->edges_[1].AsVector()).normalized();
  }
}
