// Author: Tucker Haydon

#pragma once

#include <Eigen/StdVector>
#include <Eigen/Dense>

#include "types.h"

namespace mediation_layer {
  // Abstract class representing a continuous trajectory sampled in time. At the
  // most basic view, the class is just a list of Eigen::Vectors. Provides
  // convienient methods for accessing the data.
  class Trajectory {
    private:
      // Underlying data structure. Formatted as follows:
      //   [ pos(3), vel(3), acc(3), yaw(1), time(1)]
      // Time is a floating point value measuring the seconds since the unix epoch
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

      // Position, Velocity, Acceleration
      const Eigen::Vector<double, 9> PVA(const size_t idx) const;

      // Position, Velocity, Acceleration, Yaw, Time
      const Eigen::Vector<double, 11> PVAYT(const size_t idx) const;

      // Number of samples in trajectory
      const size_t Size() const;
  };

  // Convienient type definitions
  using TrajectoryVector3D = std::vector<
    Eigen::Vector<double, 11>, 
    Eigen::aligned_allocator<Eigen::Vector<double, 11>>>;
}
