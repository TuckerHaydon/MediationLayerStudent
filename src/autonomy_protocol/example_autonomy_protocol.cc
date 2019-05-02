// Author: Tucker Haydon

#include <chrono>

#include "example_autonomy_protocol.h"

namespace game_engine {
  std::unordered_map<std::string, Trajectory> ExampleAutonomyProtocol::UpdateTrajectories() {

    // Always use the chrono::system_clock for time. Trajectories require time
    // points measured in floating point seconds from the unix epoch.
    const std::chrono::milliseconds T_chrono = std::chrono::seconds(30);
    const std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(15);
    const double dt = std::chrono::duration_cast<std::chrono::duration<double>>(dt_chrono).count();

    // Use a static function variable to log the first time this function was
    // called. The static function variable acts like a matlab persistent
    // variable, maintaining its value between function calls. The initializer 
    // is only called once.
    static const std::chrono::time_point<std::chrono::system_clock> start_chrono_time 
      = std::chrono::system_clock::now();
    static const std::chrono::time_point<std::chrono::system_clock> end_chrono_time
      = start_chrono_time + T_chrono;
    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time 
      = std::chrono::system_clock::now();

    // If at the end, return an empty map
    std::unordered_map<std::string, Trajectory> trajectory_map;
    if(current_chrono_time > end_chrono_time) {
      return trajectory_map;
    }


    const std::chrono::duration<double> remaining_chrono_time = end_chrono_time - current_chrono_time;

    // Number of samples
    const size_t N = remaining_chrono_time/dt_chrono;

    // Radius
    const double radius = 0.5;

    // Angular speed in radians/s
    constexpr double omega = 2*M_PI/30;

    const std::string& quad_name = this->friendly_names_[0];

    // Load the current position
    Eigen::Vector3d current_pos;
    this->snapshot_->Position(quad_name, current_pos);

    // Place the center of the circle 1 radius in the y-direction from the
    // starting position
    static Eigen::Vector3d circle_center = current_pos + Eigen::Vector3d(0, radius, 0);

    // Transform the current position into an angle
    const Eigen::Vector3d r = current_pos - circle_center;
    const double theta_start = std::atan2(r.y(), r.x());
  
    // TrajectoryVector3D is an std::vector object defined in the trajectory.h
    // file. It's aliased for convenience.
    TrajectoryVector3D trajectory_vector;
    for(size_t idx = 0; idx < N; ++idx) {
      // chrono::duration<double> maintains high-precision floating point time in seconds
      // use the count function to cast into floating point
      const std::chrono::duration<double> flight_chrono_time 
        = current_chrono_time.time_since_epoch() + idx * dt_chrono;
      const double flight_time = flight_chrono_time.count();
  
      // Angle in radians
      const double theta = theta_start + omega * idx * dt;
  
      // Circle 
      const double x = circle_center.x() + radius * std::cos(theta);
      const double y = circle_center.y() + radius * std::sin(theta);
      const double z = circle_center.z();
  
      // Chain rule
      const double vx = -radius * std::sin(theta) * omega;
      const double vy =  radius * std::cos(theta) * omega;
      const double vz = 0.0;
  
      // Chain rule
      const double ax = -radius * std::cos(theta) * omega * omega;
      const double ay = -radius * std::sin(theta) * omega * omega;
      const double az = 0.0;
  
      const double yaw = 0.0;
  
      // The trajectory requires the time to be specified as a floating point
      // number that measures the number of seconds since the unix epoch.
      const double time = flight_chrono_time.count();
  
      // Push an Eigen instance onto the trajectory vector
      trajectory_vector.push_back(Eigen::Matrix<double, 11, 1>(
            x,   y,   z,
            vx,  vy,  vz,
            ax,  ay,  az,
            yaw,
            time
            ));
    }
  
    // Construct a trajectory from the trajectory vector
    Trajectory trajectory(trajectory_vector);
    trajectory_map[quad_name] = trajectory; 
    

    return trajectory_map;
  }
}
