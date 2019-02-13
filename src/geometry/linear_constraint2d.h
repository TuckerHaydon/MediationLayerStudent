// Author: Tucker Haydon

#pragma once

#include "types.h"

namespace path_planning {
  // Abstraction of a linear constraint of the form AX < B, where X is
  // two-dimensional
  struct LinearConstraint2D {
    Point2D A_;
    double B_;

    LinearConstraint2D(const Point2D& A, 
                       const double B, 
                       const bool contains_origin=true) {
      // Changes the direction of the constraint depending on whether or not it
      // contains the origin
      if(true == contains_origin) {
        if(0 <= B) {
          this->A_ = A;
          this->B_ = B;
        } else {
          this->A_ = -A;
          this->B_ = -B;
        }
      } else {
        if(0 <= B) {
          this->A_ = -A;
          this->B_ = -B;
        } else {
          this->A_ = A;
          this->B_ = B;
        }

      }
    }

    Point2D IntersectionPoint(const LinearConstraint2D& other) const {
      return (
          Eigen::Matrix<double,2,2>() << 
            this->A_.transpose(), 
            other.A_.transpose()
          ).finished().inverse() 
        * Eigen::Vector2d(this->B_, other.B_);
    }

    bool Constrains(const Point2D& point) const {
      return this->A_.dot(point) - this->B_ < 1e-3;
    }

    bool ConstrainsEqual(const Point2D& point) const {
      return std::abs(this->A_.dot(point) - this->B_) < 1e-3;
    }

  };
}
