// Author: Tucker Haydon

#pragma once

#include <vector>
#include <string>

#include "trajectory_warden.h"
#include "trajectory.h"

#include "quad_state_warden.h"
#include "quad_state.h"

namespace mediation_layer {
  // A GameSnapshot is a particular view of the game that is presented to the
  // automation protocol. Typically, a team has full knowledge of the pose of
  // their quadcopters but only has limited knowledge of the pose of the
  // opponents quadcopters (i.e. a noisy position estimate).
  template <size_t T>
  class GameSnapshot {
    public:
      struct Options {

        Options() {}
      };

      GameSnapshot(
          const std::vector<std::string> friendly_names,
          const std::vector<std::string> enemy_names,
          const std::shared_ptr<QuadStateWarden<T>> quad_state_warden,
          const Options& options)
        : friendly_names_(friendly_names),
          enemy_names_(enemy_names),
          quad_state_warden_(quad_state_warden),
          options_(options) {}

      // Returns the yaw of the quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'position'.
      //
      // If the quadcopter is friendly, the position returned is accurate.
      // If the quadcopter is enemy, the position returned is corrupted by noise.
      bool Position(const std::string& quad_name, Eigen::Vector<double, T>& position);

      // Returns the orientation of the quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'orientation'. Orientation
      // is represented as a quaternion [w,x,y,z]
      //
      // If the quadcopter is friendly, the yaw returned is accurate.
      // If the quadcopter is enemy, the yaw returned is corrupted by noise.
      bool Orientation(const std::string& quad_name, Eigen::Vector<double, 4>& orientation);

      // Returns the velocity of a quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'velocity'.
      //
      // If the quadcopter is friendly, the velocity returned is accurate.
      // If the quadcopter is enemy, return false --- no access to enemy
      // velocity.
      bool Velocity(const std::string& quad_name, Eigen::Vector<double, T>& velocity);


    private:
      Options options_;
      std::vector<std::string> friendly_names_;
      std::vector<std::string> enemy_names_;
      const std::shared_ptr<QuadStateWarden<T>> quad_state_warden_;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline bool GameSnapshot<T>::Position(
      const std::string& quad_name, 
      Eigen::Vector<double, T>& position) {
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
    QuadState<T> state;
    this->quad_state_warden_.Read(quad_name, state);

    // Copy the position
    position = state.Position();

    if(true == is_enemy) {
      // TODO: Corrupt position with noise
    }

    return true;
  }

  template <size_t T>
  inline bool GameSnapshot<T>::Orientation(
      const std::string& quad_name, 
      Eigen::Vector<double, 4>& orientation) {
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
    QuadState<T> state;
    this->quad_state_warden_.Read(quad_name, state);

    // Copy the yaw
    orientation = Eigen::Vector<double, 4>(state.Orientation());

    if(true == is_enemy) {
      // TODO: Corrupt position with noise
    }

    return true;
  }

  template <size_t T>
  inline bool GameSnapshot<T>::Velocity(
      const std::string& quad_name, 
      Eigen::Vector<double, T>& velocity) {
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
    QuadState<T> state;
    this->quad_state_warden_.Read(quad_name, state);

    // Copy the velocity
    velocity = state.Velocity();

    return true;
  }

  using GameSnapshot2D = GameSnapshot<2>;
  using GameSnapshot3D = GameSnapshot<3>;
}
