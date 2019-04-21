// Author: Tucker Haydon

#pragma once

#include "autonomy_protocol.h"

#include <chrono>

namespace mediation_layer {
  class TestAP : public AutonomyProtocol {
    private:

    public:
      TestAP(
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
  std::unordered_map<std::string, Trajectory> TestAP::UpdateTrajectories() {
    std::unordered_map<std::string, Trajectory> m;

    const auto current_time = std::chrono::system_clock::now();

    const size_t N = 10;
    const std::chrono::milliseconds dt = std::chrono::milliseconds(20);
    TrajectoryVector3D hold_position_vector;
    for(size_t idx = 0; idx < N; ++idx) {
      const double time_float
          = std::chrono::duration_cast<std::chrono::duration<double>>(
              current_time.time_since_epoch() + idx*dt).count();

      hold_position_vector.push_back(Eigen::Vector<double, 11>(
            0,0,1,
            0,0,0,
            0,0,0,
            0,
            time_float 
            ));
    }

    Trajectory hold_position_trajectory(hold_position_vector);

    for(const std::string& quad_name: this->friendly_names_) {
      m[quad_name] = hold_position_trajectory;
    }

    return m;
  }
}
