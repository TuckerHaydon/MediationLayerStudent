// Author: Tucker Haydon

#pragma once

#include <vector>
#include <Eigen/Dense>

#include "types.h"

namespace mediation_layer {
  class Trajectory2D {
    private:
      std::vector<Eigen::Matrix<double, 8, 1>> data_;
  
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Trajectory2D(const std::vector<Eigen::Matrix<double, 8, 1>>& data = {}) 
        : data_(data) {}

      // Data access functions
      const Vec2D Position(const size_t idx) const;
      const Vec2D Velocity(const size_t idx) const;
      const Vec2D Acceleration(const size_t idx) const;
      const double Yaw(const size_t idx) const;
      const double Time(const size_t idx) const;
      const Eigen::Matrix<double, 6, 1> PVA(const size_t idx) const;
      const Eigen::Matrix<double, 8, 1> PVAYT(const size_t idx) const;
      const size_t Size() const;

      // Data modification
      bool Append(const Eigen::Matrix<double, 8, 1>& instance);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const size_t Trajectory2D::Size() const {
    return this->data_.size();
  }

  inline const Vec2D Trajectory2D::Position(const size_t idx) const {
    return Vec2D(this->data_[idx](0), this->data_[idx](1));
  }

  inline const Vec2D Trajectory2D::Velocity(const size_t idx) const {
    return Vec2D(this->data_[idx](2), this->data_[idx](3));
  }

  inline const Vec2D Trajectory2D::Acceleration(const size_t idx) const {
    return Vec2D(this->data_[idx](4), this->data_[idx](5));
  }

  inline const double Trajectory2D::Yaw(const size_t idx) const {
    return this->data_[idx](6);
  }

  inline const double Trajectory2D::Time(const size_t idx) const {
    return this->data_[idx](7);
  }

  inline const Eigen::Matrix<double, 6, 1> Trajectory2D::PVA(const size_t idx) const {
    return (Eigen::Matrix<double, 6, 1>() << 
        this->Position(idx),
        this->Velocity(idx),
        this->Acceleration(idx)).finished();
  }

  inline const Eigen::Matrix<double, 8, 1> Trajectory2D::PVAYT(const size_t idx) const {
    return this->data_[idx];
  }

  inline bool Trajectory2D::Append(const Eigen::Matrix<double, 8, 1>& instance) {
    this->data_.push_back(instance);
    return true;
  }
}
