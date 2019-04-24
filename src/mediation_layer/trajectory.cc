// Author: Tucker Haydon

#include "trajectory.h"

namespace mediation_layer {
  const size_t Trajectory::Size() const {
    return this->data_.size();
  }

  const Eigen::Vector<double, 3> Trajectory::Position(const size_t idx) const {
    return this->data_[idx].segment(0, 3);
  }

  const Eigen::Vector<double, 3> Trajectory::Velocity(const size_t idx) const {
    return this->data_[idx].segment(3, 3);
  }

  const Eigen::Vector<double, 3> Trajectory::Acceleration(const size_t idx) const {
    return this->data_[idx].segment(6,3);
  }

  const double Trajectory::Yaw(const size_t idx) const {
    return this->data_[idx](9);
  }

  const double Trajectory::Time(const size_t idx) const {
    return this->data_[idx](10);
  }

  const Eigen::Vector<double, 9> Trajectory::PVA(const size_t idx) const {
    return this->data_[idx].segment(0,9);
  }

  const Eigen::Vector<double, 11> Trajectory::PVAYT(const size_t idx) const {
    return this->data_[idx];
  }
}
