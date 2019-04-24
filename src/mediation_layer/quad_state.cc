// Author: Tucker Haydon

#include "quad_state.h"

namespace mediation_layer {
  Eigen::Vector<double, 3> QuadState::Position() const {
    return data_.segment(0,3);
  }

  Eigen::Vector<double, 3> QuadState::Velocity() const {
    return data_.segment(3,3);
  }

  Eigen::Vector<double, 4> QuadState::Orientation() const {
    return data_.segment(6,4);
  }

  Eigen::Vector<double, 3> QuadState::Twist() const {
    return data_.segment(10,3);
  }
}

