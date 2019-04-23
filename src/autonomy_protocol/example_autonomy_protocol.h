// Author: Tucker Haydon

#pragma once

#include "autonomy_protocol.h"

#include <chrono>

namespace mediation_layer {
  // The ExampleAutonomyProtocol is a class that demonstrates how to create a
  // trajectory for a quadcopter to follow. The example protocol instructs a
  // quadcopter to fly in a circle of radius two at an altitude of one meter.
  //
  // The trajectory with be specified by parameterizing the circle with time.
  // The quadcopter must complete a revolution every five seconds. 
  class ExampleAutonomyProtocol : public AutonomyProtocol {
    private:

    public:
      ExampleAutonomyProtocol(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot> snapshot,
          const std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
          const Map3D& map3d,
          std::map<
            std::string, 
            Eigen::Vector<double, 3>, 
            std::less<std::string>, 
            Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector<double, 3>>>> balloon_map)
        : AutonomyProtocol(
            friendly_names,
            enemy_names,
            snapshot,
            trajectory_warden_out,
            map3d,
            balloon_map) {}

      std::unordered_map<std::string, Trajectory> UpdateTrajectories() override;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  std::unordered_map<std::string, Trajectory> ExampleAutonomyProtocol::UpdateTrajectories() {

    // Use a static function variable to ensure that one and only one trajectory
    // is published. Static variables are not destroyed when the function leaves
    // scope, instead they keep their last value and may be accessed again in
    // the next function call. Static variables are only initialized once (it
    // will only be set to false once). These variables may be thought of the
    // C++ equivalent to Matlab's persistent variable.
    //
    // If the trajectory has already been specified, return an empty map. By
    // returning an empty map, we instruct the controller to follow the previous
    // trajectory and hold the final position.
    static bool trajectory_already_specified = false;
    if(true == trajectory_already_specified) {
      std::unordered_map<std::string, Trajectory> m;
      return m;
    } else {
      trajectory_already_specified = true;
    }

    // Always use the chrono::system_clock for time. Trajectories require time
    // points measured in floating point seconds from the unix epoch.
    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time 
      = std::chrono::system_clock::now();
    const std::chrono::milliseconds T = std::chrono::seconds(30);
    const std::chrono::milliseconds dt = std::chrono::milliseconds(20);

    // Number of samples
    const size_t N = T/dt;

    // Angular speed in radians/s
    const double omega = 2*M_PI/5.0;

    // TrajectoryVector3D is an std::vector object defined in the trajectory.h
    // file. It's aliased for convenience.
    TrajectoryVector3D trajectory_vector;
    for(size_t idx = 0; idx < N; ++idx) {
      // chrono::duration<double> maintains high-precision floating point time in seconds
      // use the count function to cast into floating point
      const std::chrono::duration<double> local_chrono_time = idx * dt;
      const double local_time = local_chrono_time.count();

      // Angle in radians
      const double theta = local_time / omega;

      // Circle centered at (1,1,1)
      const double x = 1.0 + std::cos(theta);
      const double y = 1.0 + std::sin(theta);
      const double z = 1.0;

      const double vx = -std::sin(theta);
      const double vy =  std::cos(theta);
      const double vz = 0.0;

      const double ax = -std::cos(theta);
      const double ay = -std::sin(theta);
      const double az = 0.0;

      const double yaw = 0.0;

      // The trajectory requires the time to be specified as a floating point
      // number that measures the number of seconds since the unix epoch.
      const std::chrono::duration<double> global_time_chrono 
        = current_chrono_time.time_since_epoch() + local_chrono_time;
      const double global_time = global_time_chrono.count();

      // Push an Eigen instance onto the trajectory vector
      trajectory_vector.push_back(Eigen::Vector<double, 11>(
            x,y,z,
            vx,vy,vz,
            ax,ay,az,
            yaw,
            global_time
            ));
    }

    // Construct a trajectory from the trajectory vector
    Trajectory trajectory(trajectory_vector);

    // Assign every friendly quad to execute this trajectory. Really there is
    // only one quad for the current game so it doesn't matter.
    std::unordered_map<std::string, Trajectory> m;
    for(const std::string& quad_name: this->friendly_names_) {
      m[quad_name] = trajectory;
    }

    return m;
  }
}
