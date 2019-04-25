// Author: Tucker Haydon

#pragma once

#include <memory>
#include <string>

#include "trajectory.h"
#include "map3d.h"
#include "quad_state_warden.h"

namespace game_engine {
  // The TrajectoryVetter determines if a Trajectory complies with a set of
  // specified requirements. The requirements are:
  //   1) A quad following the trajectory will not exceed the boundaries of the
  //      map
  //   2) A quad following the trajectory will not pass through or touch any
  //      obstacles
  //   3) A quad following the trajectory will not exceed the maximum specified
  //      velocity.
  //   4) A quad following the trajectory will not exceed the maximum
  //      specified acceleration.
  //   5) The quad following the trajectory is no more that a specified maximum
  //      distance from the first point in the trajectory
  //   6) The time between trajectory samples is no more than a specified
  //      maximum delta-time
  //
  // TODO: Adjust max_distance_from_current_position
  class TrajectoryVetter {
    public:
      struct Options {
        // Maximum allowed velocity in any direction in m/s
        double max_velocity_magnitude = 2.0;

        // Maximum allowed acceleration in any direction in m/s^2
        double max_acceleration_magnitude = 0.4;

        // The maximum distance from a quad's current position that a
        // trajectory's first point may deviate in meters
        double max_distance_from_current_position = 1.00;

        // The maximum time between trajectory samples in seconds
        double max_delta_t = 0.020;

        Options() {}
      };

      TrajectoryVetter(const Options& options = Options())
        : options_(options) {}

      // Determines if a trajectory meets the trajectory requirements laid out
      // in the documentation
      bool Vet(
          const Trajectory& trajectory,
          const Map3D& map,
          const std::shared_ptr<QuadStateWarden> quad_state_warden,
          const std::string& quad_name
          ) const;

    private:
      Options options_;

  };
}
