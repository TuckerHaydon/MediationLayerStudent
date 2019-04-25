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
      Eigen::Matrix<double, 13, 1> data_;

    public:
      // Required by Eigen
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      QuadState(const Eigen::Matrix<double, 13, 1>& data 
          = Eigen::Matrix<double, 13, 1>::Zero())
        : data_(data) {}

      // Getters
      Eigen::Vector3d Position() const;
      Eigen::Vector3d Velocity() const;
      Eigen::Vector4d Orientation() const;
      Eigen::Vector3d Twist() const;
  };
}
