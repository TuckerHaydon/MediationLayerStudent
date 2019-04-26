// Author: Tucker Haydon

#include <chrono>
#include <thread>

#include "balloon_watchdog.h"
#include "balloon_status.h"

namespace game_engine {
  void BalloonWatchdog::Run(
      std::shared_ptr<BalloonStatusPublisherNode> balloon_status_publisher,
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::vector<std::string>& quad_names,
      const Eigen::Vector3d& balloon_position) {

    // Data to be populated when balloon is popped
    bool balloon_popped = false;
    std::chrono::system_clock::time_point balloon_pop_time
      = std::chrono::system_clock::now();
    std::string quad_popper = "null";

    // Main loop
    while(true == this->ok_) {
      for(const std::string& quad_name: quad_names) {
        QuadState quad_state;
        quad_state_warden->Read(quad_name, quad_state);

        const Eigen::Vector3d quad_pos = quad_state.Position();
        const double distance_to_balloon = (quad_pos - balloon_position).norm();

        if(this->options_.pop_distance >= distance_to_balloon) {
          if(false == balloon_popped) {
            balloon_popped = true;
            balloon_pop_time = std::chrono::system_clock::now();
            quad_popper = quad_name;
          }
        }
      }

      // Publish
      BalloonStatus balloon_status {
        .popped = balloon_popped,
        .popper = quad_popper,
        .pop_time = balloon_pop_time
      };

      balloon_status_publisher->Publish(balloon_status);

      // 10 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void BalloonWatchdog::Stop() {
    this->ok_ = false;
  }

}
