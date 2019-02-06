// Author: Tucker Haydon

#ifndef GEOMETRY_POINT2D_H
#define GEOMETRY_POINT2D_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace path_planning {
  typedef Eigen::Vector2d Point2D;

  // /*
  //  * Encapsulates information about a 2D point
  //  */
  // class Point2D {
  //   private:
  //     Eigen::Vector2d data_;

  //   public:
  //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //     Point2D(const double x, const double y) 
  //       : Point2D(Eigen::Vector2d(x,y)) {}

  //     Point2D(const Eigen::Vector2d& data = Eigen::Vector2d())
  //       : data_(data) {};

  //     const Eigen::Vector2d& Data() const;
  //     bool SetData(const Eigen::Vector2d& data);

  //     double X() const;
  //     double Y() const;

  //     double Norm() const;

  //     Point2D operator-(const Point2D& rhs) const;
  //     Point2D operator+(const Point2D& rhs) const;
  //     bool operator==(const Point2D& rhs) const;
  // };

  // //  ******************
  // //  * IMPLEMENTATION *
  // //  ******************
  // inline const Eigen::Vector2d& Point2D::Data() const {
  //   return this->data_;
  // }

  // inline bool Point2D::SetData(const Eigen::Vector2d& data) {
  //   this->data_ = data;
  //   return true;
  // }

  // inline Point2D Point2D::operator-(const Point2D& rhs) const {
  //   return Point2D(this->data_ - rhs.data_);
  // }

  // inline Point2D Point2D::operator+(const Point2D& rhs) const {
  //   return Point2D(this->data_ + rhs.data_);
  // }

  // inline bool Point2D::operator==(const Point2D& rhs) const {
  //   return this->data_.isApprox(rhs.data_, 1e-3);
  // }

  // inline double Point2D::X() const {
  //   return this->data_(0);
  // }

  // inline double Point2D::Y() const {
  //   return this->data_(1);
  // }

  // inline double Point2D::Norm() const {
  //   return this->data_.norm();
  // } 
}

#endif
