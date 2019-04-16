// Author: Tucker Haydon

#pragma once

#include <Eigen/StdVector>
#include <Eigen/Dense>

#include "types.h"

namespace mediation_layer {

  // Abstract class encapsulating a trajectory through time. At the most basic
  // view, the class is just a list of Eigen::Vectors. Provides convienient
  // methods for accessing the data.
  class Trajectory {
    private:
      // Underlying data structure. Formatted as follows:
      //   [ pos(3), vel(3), acc(3), yaw(1), time(1)]
      std::vector<
        Eigen::Vector<double, 11>, 
        Eigen::aligned_allocator<Eigen::Vector<double, 11>>> data_;
  
    public:
      // Required by Eigen
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Constructor. Data must be passed in the following format: 
      //   [ pos, vel, acc, yaw, time]
      Trajectory(const std::vector<
          Eigen::Vector<double, 11>, 
          Eigen::aligned_allocator<Eigen::Vector<double, 11>>>& data = {}) 
        : data_(data) {}

      // Data access functions
      const Eigen::Vector<double, 3> Position(const size_t idx) const;
      const Eigen::Vector<double, 3> Velocity(const size_t idx) const;
      const Eigen::Vector<double, 3> Acceleration(const size_t idx) const;
      const double Yaw(const size_t idx) const;
      const double Time(const size_t idx) const;
      const Eigen::Vector<double, 9> PVA(const size_t idx) const;
      const Eigen::Vector<double, 11> PVAYT(const size_t idx) const;
      const size_t Size() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const size_t Trajectory::Size() const {
    return this->data_.size();
  }

  inline const Eigen::Vector<double, 3> Trajectory::Position(const size_t idx) const {
    return this->data_[idx].segment(0, 3);
  }

  inline const Eigen::Vector<double, 3> Trajectory::Velocity(const size_t idx) const {
    return this->data_[idx].segment(3, 3);
  }

  inline const Eigen::Vector<double, 3> Trajectory::Acceleration(const size_t idx) const {
    return this->data_[idx].segment(6,3);
  }

  inline const double Trajectory::Yaw(const size_t idx) const {
    return this->data_[idx](9);
  }

  inline const double Trajectory::Time(const size_t idx) const {
    return this->data_[idx](10);
  }

  inline const Eigen::Vector<double, 9> Trajectory::PVA(const size_t idx) const {
    return this->data_[idx].segment(0,9);
  }

  inline const Eigen::Vector<double, 11> Trajectory::PVAYT(const size_t idx) const {
    return this->data_[idx];
  }

  // Convienient type definitions
  using TrajectoryVector3D = std::vector<
    Eigen::Vector<double, 11>, 
    Eigen::aligned_allocator<Eigen::Vector<double, 11>>>;
}
