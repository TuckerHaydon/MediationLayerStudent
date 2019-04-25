// Author: Tucker Haydon

#pragma once

#include <vector>
#include <string>

#include "quad_state_warden.h"
#include "quad_state.h"

namespace game_engine {
  // A GameSnapshot is a particular view of the game that is presented to the
  // autonomy protocol. Typically, a team has full knowledge of the pose of
  // their quadcopters but only has limited knowledge of the pose of the
  // opponents quadcopters (i.e. a noisy position estimate).
  //
  // TODO: Corrupt enemy states with noise
  class GameSnapshot {
    public:
      struct Options {

        Options() {}
      };

      GameSnapshot(
          const std::vector<std::string> friendly_names,
          const std::vector<std::string> enemy_names,
          const std::shared_ptr<QuadStateWarden> quad_state_warden,
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
      bool Position(const std::string& quad_name, Eigen::Vector3d& position);

      // Returns the orientation of the quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'orientation'. Orientation
      // is represented as a quaternion [w,x,y,z]
      //
      // If the quadcopter is friendly, the yaw returned is accurate.
      // If the quadcopter is enemy, the yaw returned is corrupted by noise.
      bool Orientation(const std::string& quad_name, Eigen::Vector4d& orientation);

      // Returns the velocity of a quadcopter. If quad_name is invalid, returns
      // false, else returns true. Stores the data in 'velocity'.
      //
      // If the quadcopter is friendly, the velocity returned is accurate.
      // If the quadcopter is enemy, return false --- no access to enemy
      // velocity.
      bool Velocity(const std::string& quad_name, Eigen::Vector3d& velocity);


    private:
      Options options_;
      std::vector<std::string> friendly_names_;
      std::vector<std::string> enemy_names_;
      const std::shared_ptr<QuadStateWarden> quad_state_warden_;
  };
}
