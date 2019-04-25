// Author: Tucker Haydon

#include "game_snapshot.h"

namespace game_engine {
  bool GameSnapshot::Position(
      const std::string& quad_name, 
      Eigen::Vector3d& position) {
    const bool is_friend = (std::find(
          this->friendly_names_.begin(), 
          this->friendly_names_.end(), 
          quad_name) != this->friendly_names_.end());

    const bool is_enemy = (std::find(
          this->enemy_names_.begin(), 
          this->enemy_names_.end(), 
          quad_name) != this->enemy_names_.end());

    // Invalid quad name
    if(false == (is_enemy || is_friend)) {
      return false;
    }

    // Read from the warden
    QuadState state;
    this->quad_state_warden_->Read(quad_name, state);

    // Copy the position
    position = state.Position();

    if(true == is_enemy) {
      // TODO: Corrupt position with noise
    }

    return true;
  }

  bool GameSnapshot::Orientation(
      const std::string& quad_name, 
      Eigen::Vector4d& orientation) {
    const bool is_friend = (std::find(
          this->friendly_names_.begin(), 
          this->friendly_names_.end(), 
          quad_name) != this->friendly_names_.end());

    const bool is_enemy = (std::find(
          this->enemy_names_.begin(), 
          this->enemy_names_.end(), 
          quad_name) != this->enemy_names_.end());

    // Invalid quad name
    if(false == (is_enemy || is_friend)) {
      return false;
    }

    // Read from the warden
    QuadState state;
    this->quad_state_warden_->Read(quad_name, state);

    // Copy the yaw
    orientation = Eigen::Vector4d(state.Orientation());

    if(true == is_enemy) {
      // TODO: Corrupt position with noise
    }

    return true;
  }

  bool GameSnapshot::Velocity(
      const std::string& quad_name, 
      Eigen::Vector3d& velocity) {
    const bool is_friend = (std::find(
          this->friendly_names_.begin(), 
          this->friendly_names_.end(), 
          quad_name) != this->friendly_names_.end());

    const bool is_enemy = (std::find(
          this->enemy_names_.begin(), 
          this->enemy_names_.end(), 
          quad_name) != this->enemy_names_.end());

    // Invalid quad name or not friend
    if(false == is_friend) {
      return false;
    }

    // Read from the warden
    QuadState state;
    this->quad_state_warden_->Read(quad_name, state);

    // Copy the velocity
    velocity = state.Velocity();

    return true;
  }

}
