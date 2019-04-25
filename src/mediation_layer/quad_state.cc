// Author: Tucker Haydon

#include "quad_state.h"

namespace game_engine {
  Eigen::Vector3d QuadState::Position() const {
    return data_.segment(0,3);
  }

  Eigen::Vector3d QuadState::Velocity() const {
    return data_.segment(3,3);
  }

  Eigen::Vector4d QuadState::Orientation() const {
    return data_.segment(6,4);
  }

  Eigen::Vector3d QuadState::Twist() const {
    return data_.segment(10,3);
  }
}

