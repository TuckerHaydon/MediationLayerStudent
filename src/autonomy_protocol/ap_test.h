// Author: Tucker Haydon

#pragma once

#include "autonomy_protocol.h"

namespace mediation_layer {
  class TestAP : public AutonomyProtocol {
    private:

    public:
      TestAP(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot> snapshot,
          const std::shared_ptr<TrajectoryWarden> trajectory_warden_out)
        : AutonomyProtocol(
            friendly_names,
            enemy_names,
            snapshot,
            trajectory_warden_out) {}

      std::unordered_map<std::string, Trajectory> UpdateTrajectories(
          std::shared_ptr<GameSnapshot> snapshot,
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names) override;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  std::unordered_map<std::string, Trajectory> TestAP::UpdateTrajectories(
      std::shared_ptr<GameSnapshot> snapshot,
      const std::vector<std::string>& friendly_names,
      const std::vector<std::string>& enemy_names) {
    std::unordered_map<std::string, Trajectory> m;
    return m;
  }
}
