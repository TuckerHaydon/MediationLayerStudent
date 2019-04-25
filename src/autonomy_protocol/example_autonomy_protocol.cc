// Author: Tucker Haydon

#include <chrono>

#include "example_autonomy_protocol.h"

namespace game_engine {
  std::unordered_map<std::string, Trajectory> ExampleAutonomyProtocol::UpdateTrajectories() {

    // Always use the chrono::system_clock for time. Trajectories require time
    // points measured in floating point seconds from the unix epoch.
    const std::chrono::milliseconds T = std::chrono::seconds(30);
    const std::chrono::milliseconds dt = std::chrono::milliseconds(20);
    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time 
      = std::chrono::system_clock::now();

    // Use a static function variable to log the first time this function was
    // called. The static function variable acts like a matlab persistent
    // variable, maintaining its value between function calls. The initializer 
    // is only called once.
    static const std::chrono::time_point<std::chrono::system_clock> start_chrono_time 
      = std::chrono::system_clock::now();
    static const std::chrono::time_point<std::chrono::system_clock> end_chrono_time
      = start_chrono_time + T;

    // If at the end, return an empty map
    if(current_chrono_time > end_chrono_time) {
      return std::unordered_map<std::string, Trajectory>();
    }

    const std::chrono::duration<double> remaining_chrono_time = end_chrono_time - current_chrono_time;
    const std::chrono::duration<double> current_flight_chrono_time = current_chrono_time - start_chrono_time;

    // Number of samples
    const size_t N = remaining_chrono_time/dt;

    // Angular speed in radians/s
    constexpr double omega = 2*M_PI/5.0;

    // TrajectoryVector3D is an std::vector object defined in the trajectory.h
    // file. It's aliased for convenience.
    TrajectoryVector3D trajectory_vector;
    for(size_t idx = 0; idx < N; ++idx) {
      // chrono::duration<double> maintains high-precision floating point time in seconds
      // use the count function to cast into floating point
      const std::chrono::duration<double> flight_chrono_time = current_flight_chrono_time + idx * dt;
      const double flight_time = flight_chrono_time.count();

      // Angle in radians
      const double theta = flight_time / omega;

      // Circle centered at (1,2,1)
      const double x = 1.0 + std::cos(theta);
      const double y = 2.0 + std::sin(theta);
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
        = current_chrono_time.time_since_epoch() + flight_chrono_time;
      const double global_time = global_time_chrono.count();

      // Push an Eigen instance onto the trajectory vector
      trajectory_vector.push_back(Eigen::Matrix<double, 11, 1>(
            x,   y,   z,
            vx,  vy,  vz,
            ax,  ay,  az,
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
