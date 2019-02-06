// Author: Tucker Haydon

#ifndef GEOMETRY_POINT3D_H
#define GEOMETRY_POINT3D_H

#include <Eigen/Geometry>

namespace path_planning {
  /*
   * Encapsulates information about a 3D point
   */
  class Point3D {
    private:
      Eigen::Vector3d data_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Point3D(const double x, const double y, const double z)
        : Point3D(Eigen::Vector3d(x,y,z)) {}

      Point3D(const Eigen::Vector3d& data = Eigen::Vector3d())
        : data_(data) {}

      const Eigen::Vector3d& Data() const;
      bool SetData(const Eigen::Vector3d& data);

      Point3D operator-(const Point3D& rhs) const;
      Point3D operator+(const Point3D& rhs) const;
      Point3D Cross(const Point3D& rhs) const;
      double Dot(const Point3D& rhs) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const Eigen::Vector3d& Point3D::Data() const {
    return this->data_;
  }

  inline bool Point3D::SetData(const Eigen::Vector3d& data) {
    this->data_ = data;
    return true;
  }

  inline Point3D Point3D::operator-(const Point3D& rhs) const {
    return Point3D(this->data_ - rhs.data_);
  }

  inline Point3D Point3D::operator+(const Point3D& rhs) const {
    return Point3D(this->data_ + rhs.data_);
  }

  inline Point3D Point3D::Cross(const Point3D& rhs) const {
    return Point3D(this->data_.cross(rhs.data_));
  }

  inline double Point3D::Dot(const Point3D& rhs) const {
    return this->data_.dot(rhs.data_);
  }
}

#endif
