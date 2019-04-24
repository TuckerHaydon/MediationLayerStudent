// Author: Tucker Haydon

#pragma once

#include "autonomy_protocol.h"

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
}
