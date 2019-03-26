// Author: Tucker Haydon

#pragma once

// #include <vector>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include "types.h"

namespace mediation_layer {

  // Abstract class encapsulating a trajectory through time. At the most basic
  // view, the class is just a list of Eigen::Vectors. Provides convienient
  // methods for accessing the data.
  template <size_t T>
  class Trajectory {
    private:
      // Underlying data structure. Formatted as follows:
      //   [ pos(T), vel(T), acc(T), yaw(1), time(1)]
      const std::vector<
        Eigen::Vector<double, 3*T + 2>, 
        Eigen::aligned_allocator<Eigen::Vector<double, 3*T + 2>>> data_;
  
    public:
      // Required by Eigen
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Constructor. Data must be passed in the following format: 
      //   [ pos, vel, acc, yaw, time]
      Trajectory(const std::vector<
          Eigen::Vector<double, 3*T + 2>, 
          Eigen::aligned_allocator<Eigen::Vector<double, 3*T + 2>>>& data = {}) 
        : data_(data) {}

      // Data access functions
      const Eigen::Vector<double, T> Position(const size_t idx) const;
      const Eigen::Vector<double, T> Velocity(const size_t idx) const;
      const Eigen::Vector<double, T> Acceleration(const size_t idx) const;
      const double Yaw(const size_t idx) const;
      const double Time(const size_t idx) const;
      const Eigen::Vector<double, 3*T> PVA(const size_t idx) const;
      const Eigen::Vector<double, 3*T + 2> PVAYT(const size_t idx) const;
      const size_t Size() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template<size_t T>
  inline const size_t Trajectory<T>::Size() const {
    return this->data_.size();
  }

  template<size_t T>
  inline const Eigen::Vector<double, T> Trajectory<T>::Position(const size_t idx) const {
    return this->data_[idx].segment(0*T,T);
  }

  template<size_t T>
  inline const Eigen::Vector<double, T> Trajectory<T>::Velocity(const size_t idx) const {
    return this->data_[idx].segment(1*T,T);
  }

  template<size_t T>
  inline const Eigen::Vector<double, T> Trajectory<T>::Acceleration(const size_t idx) const {
    return this->data_[idx].segment(2*T,T);
  }

  template<size_t T>
  inline const double Trajectory<T>::Yaw(const size_t idx) const {
    return this->data_[idx](3*T);
  }

  template<size_t T>
  inline const double Trajectory<T>::Time(const size_t idx) const {
    return this->data_[idx](3*T+1);
  }

  template<size_t T>
  inline const Eigen::Vector<double, 3*T> Trajectory<T>::PVA(const size_t idx) const {
    return this->data_[idx].segment(0,3*T);
  }

  template<size_t T>
  inline const Eigen::Vector<double, 3*T + 2> Trajectory<T>::PVAYT(const size_t idx) const {
    return this->data_[idx];
  }

  // Convienient type definitions
  using TrajectoryVector2D = std::vector<
    Eigen::Vector<double, 8>, 
    Eigen::aligned_allocator<Eigen::Vector<double, 8>>>;
  using TrajectoryVector3D = std::vector<
    Eigen::Vector<double, 11>, 
    Eigen::aligned_allocator<Eigen::Vector<double, 11>>>;

  using Trajectory2D = Trajectory<2>;
  using Trajectory3D = Trajectory<3>;
}
