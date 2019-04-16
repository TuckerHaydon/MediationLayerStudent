// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>

namespace mediation_layer {
  // Abstract class encapsulating the current state of an element in the
  // mediation layer
  class QuadState {
    private:
      // The structure of the state is:
      //   [pos(3), vel(3), q(4), e_dot(3)]
      Eigen::Vector<double, 13> data_;

    public:
      // Required by Eigen
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      QuadState(const Eigen::Vector<double, 13>& data 
          = Eigen::Vector<double, 13>::Zero())
        : data_(data) {}

      // Getters
      Eigen::Vector<double, 3> Position() const;
      Eigen::Vector<double, 3> Velocity() const;
      Eigen::Vector<double, 4> Orientation() const;
      Eigen::Vector<double, 3> Twist() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Eigen::Vector<double, 3> QuadState::Position() const {
    return data_.segment(0,3);
  }

  inline Eigen::Vector<double, 3> QuadState::Velocity() const {
    return data_.segment(3,3);
  }

  inline Eigen::Vector<double, 4> QuadState::Orientation() const {
    return data_.segment(6,4);
  }

  inline Eigen::Vector<double, 3> QuadState::Twist() const {
    return data_.segment(10,3);
  }
}
