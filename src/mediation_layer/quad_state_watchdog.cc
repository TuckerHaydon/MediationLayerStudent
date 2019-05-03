// Author: Tucker Haydon

#include "quad_state_watchdog.h"

#include <thread>
#include <chrono>
#include <Eigen/Core>

namespace game_engine {

  void QuadStateWatchdog::Run(
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::vector<std::string>& quad_names,
      std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
      const Map3D map) {

    // Inflate the map by min_distance. This creates a new map whose obstacles
    // have been expanded by min_distance and whose boundaries have been shrunk
    // by min_distance. After inflating the map, the quadcopter may be treated
    // as a point-particle. To determine if the quadcopter is within an
    // l-infinity distance of min_distance from any obstacle, simpy check
    // whether the quad's center point is intersecting any of the inflate
    // obstacles.
    const Map3D inflated_map = map.Inflate(this->options_.min_distance);

    while(this->ok_) {
      for(const std::string& quad_name: quad_names) {
        // Read in the curret state
        QuadState current_state;
        quad_state_warden->Read(quad_name, current_state);

        // Get the current position
        Eigen::Vector3d current_position = current_state.Position();

        // Evaluate whether the current position intersects an obstacle
        bool infraction_occurred = 
             !inflated_map.IsFreeSpace(current_position) 
          || !inflated_map.Contains(current_position);

        if(true == infraction_occurred) {
          std::cerr 
            << quad_name 
            << " has violated the obstacle/boundary constraints. Current position is: " 
            << current_position.transpose() << std::endl;
          quad_state_watchdog_status->Write(quad_name, true);
        }
      }
      // 20 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

  }

  void QuadStateWatchdog::Stop() {
    this->ok_ = false;
  }

}
