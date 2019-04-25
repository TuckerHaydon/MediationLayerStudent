// Author: Tucker Haydon

#pragma once

#include <atomic>
#include <string>

#include "trajectory_warden.h"
#include "quad_state_warden.h"

namespace game_engine { 
  // The mediation layer is a software layer that mediates user input to ensure
  // that the trajectories provided to quadcopters are safe. During the
  // machine games, two user are allowed to specify trajectories for
  // quadcopters. The user-provided trajectories may not be safe --- the
  // trajetories might cause quads to fly into each other, walls, or other
  // obstacles in the environment. 
  //
  // TODO: Modify trajectories to prevent quads from flying into obstacles
  // TODO: Vet trajectories and ensure that they do not exceed prescribed bounds
  class MediationLayer {
    private:
      volatile std::atomic_bool ok_{true};

      // Transfers data associated with key from trajectory_warden_in to
      // trajectory_warden_out
      void TransferData(
          const std::string& key,
          std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
          std::shared_ptr<TrajectoryWarden> trajectory_warden_out);

    public:
      MediationLayer() {}

      // Run the mediation layer.
      //
      // Note: These values are intentionally copied
      void Run(
          std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
          std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
          std::shared_ptr<QuadStateWarden> state_warden);

      // Stop this thread and all sub-threads
      void Stop();

  };
}
