// Author: Tucker Haydon

#pragma once

#include <atomic>

#include "trajectory_warden.h"
#include "quad_state_warden.h"

namespace mediation_layer { 
  // The mediation layer is a software layer that mediates user input to ensure
  // that the trajectories provided to quadcopters are safe. During the
  // machine games, two user are allowed to specify trajectories for
  // quadcopters. The user-provided trajectories may not be safe --- the
  // trajetories might cause quads to fly into each other, walls, or other
  // obstacles in the environment. 
  //
  // TODO: Modify trajectories to prevent quads from flying into them
  // TODO: Change hard-coded key 'phoenix'
  class MediationLayer {
    private:
      volatile std::atomic_bool ok_{true};

    public:
      MediationLayer() {}

      // Run the mediation layer.
      //
      // Note: These values are intentionally copied
      void Run(
          std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
          std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
          std::shared_ptr<QuadStateWarden> state_warden);

      void Stop();

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline void MediationLayer::Run(
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
      std::shared_ptr<QuadStateWarden> state_warden) {

    while(this->ok_) {
      const std::string key = "phoenix";
      Trajectory trajectory;
      trajectory_warden_in->Await(key, trajectory);
      trajectory_warden_out->Write(key, trajectory);
    }
  }

  inline void MediationLayer::Stop() {
    this->ok_ = false;
  }
}
