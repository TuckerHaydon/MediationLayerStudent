// Author: Tucker Haydon

#pragma once

#include "autonomy_protocol.h"

namespace mediation_layer {
  class StudentAutonomyProtocol : public AutonomyProtocol {
    private:

    public:
      StudentAutonomyProtocol(
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
}
