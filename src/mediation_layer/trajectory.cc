// Author: Tucker Haydon

#include "trajectory.h"

namespace mediation_layer {
  const size_t Trajectory::Size() const {
    return this->data_.size();
  }

  const Eigen::Vector3d Trajectory::Position(const size_t idx) const {
    return this->data_[idx].segment(0, 3);
  }

  const Eigen::Vector3d Trajectory::Velocity(const size_t idx) const {
    return this->data_[idx].segment(3, 3);
  }

  const Eigen::Vector3d Trajectory::Acceleration(const size_t idx) const {
    return this->data_[idx].segment(6,3);
  }

  const double Trajectory::Yaw(const size_t idx) const {
    return this->data_[idx](9);
  }

  const double Trajectory::Time(const size_t idx) const {
    return this->data_[idx](10);
  }

  const Eigen::Matrix<double, 9, 1> Trajectory::PVA(const size_t idx) const {
    return this->data_[idx].segment(0,9);
  }

  const Eigen::Matrix<double, 11, 1> Trajectory::PVAYT(const size_t idx) const {
    return this->data_[idx];
  }
}
