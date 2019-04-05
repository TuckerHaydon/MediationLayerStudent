// Author: Tucker Haydon

#pragma once

#include <vector>
#include <string>

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
          const std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_in,
          const Options& options)
        : friendly_names_(friendly_names),
          enemy_names_(enemy_names),
          trajectory_warden_in_(trajectory_warden_in),
          options_(options) {}

      // Returns the yaw of the quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'position'.
      //
      // If the quadcopter is friendly, the position returned is accurate.
      // If the quadcopter is enemy, the position returned is corrupted by noise.
      bool Position(const std::string& quad_name, Eigen::Vector<double, T>& position);

      // Returns the yaw of the quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'yaw'.
      //
      // If the quadcopter is friendly, the yaw returned is accurate.
      // If the quadcopter is enemy, the yaw returned is corrupted by noise.
      bool Yaw(const std::string& quad_name, Eigen::Vector<double, 1>& yaw);

      // Returns the velocity of a quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'velocity'.
      //
      // If the quadcopter is friendly, the velocity returned is accurate.
      // If the quadcopter is enemy, return false --- no access to enemy
      // velocity.
      bool Velocity(const std::string& quad_name, Eigen::Vector<double, T>& velocity);

      // Returns the acceleration of a quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'acceleration'.
      //
      // If the quadcopter is friendly, the acceleration returned is accurate.
      // If the quadcopter is enemy, return false --- no access to enemy
      // acceleration.
      bool Acceleration(const std::string& quad_name, Eigen::Vector<double, T>);


    private:
      Options options_;
      std::vector<std::string> friendly_names_;
      std::vector<std::string> enemy_names_;
      const std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_in_;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool GameSnapshot::Position(
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
    Trajectory<T> trajectory;
    this->trajectory_warden_in_.Read(quad_name, trajectory);

    // Copy the position
    position = trajectory.Position(0);

    if(true == is_enemy) {
      // TODO: Corrupt position with noise
    }

    return true;
  }

  inline bool GameSnapshot::Yaw(
      const std::string& quad_name, 
      Eigen::Vector<double, 1>& yaw) {
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
    Trajectory<T> trajectory;
    this->trajectory_warden_in_.Read(quad_name, trajectory);

    // Copy the yaw
    yaw = Eigen::Vector<double, 1>(trajectory.Yaw(0));

    if(true == is_enemy) {
      // TODO: Corrupt position with noise
    }

    return true;
  }

  inline bool GameSnapshot::Velocity(
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
    Trajectory<T> trajectory;
    this->trajectory_warden_in_.Read(quad_name, trajectory);

    // Copy the velocity
    velocity = trajectory.Velocity(0);

    return true;
  }

  inline bool GameSnapshot::Acceleration(
      const std::string& quad_name, 
      Eigen::Vector<double, T>& acceleration) {
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
    Trajectory<T> trajectory;
    this->trajectory_warden_in_.Read(quad_name, trajectory);

    // Copy the velocity
    acceleration = trajectory.Acceleration(0);

    return true;
  }

  using GameSnapshot2D = GameSnapshot<2>;
  using GameSnapshot3D = GameSnapshot<3>;
}
