// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>

namespace mediation_layer {
  // Abstract class encapsulating the current state of an element in the
  // mediation layer
  template <size_t T>
  class QuadState {
    private:
      // The structure of the state is:
      //   [pos(T), vel(T), q(4), e_dot(3)]
      const Eigen::Vector<double, 2*T + 7> data_;

    public:
      // Required by Eigen
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      QuadState(const Eigen::Vector<double, 2*T + 7>& data)
        : data_(data) {}

      // Getters
      Eigen::Vector<double, T> Position() const;
      Eigen::Vector<double, T> Velocity() const;
      Eigen::Vector<double, 4> Orientation() const;
      Eigen::Vector<double, 3> Twist() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  Eigen::Vector<double, T> QuadState::Position() const {
    return data_.segment(0*T,T);
  }

  template <size_t T>
  Eigen::Vector<double, T> QuadState::Velocity() const {
    return data_.segment(1*T,T);
  }

  template <size_t T>
  Eigen::Vector<double, T> QuadState::Orientation() const {
    return data_.segment(2*T,4);
  }

  template <size_t T>
  Eigen::Vector<double, T> QuadState::Twist() const {
    return data_.segment(2*T+4,3);
  }
}
