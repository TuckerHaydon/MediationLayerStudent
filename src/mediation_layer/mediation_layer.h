// Author: Tucker Haydon

#pragma once

#include <atomic>

#include "trajectory_warden.h"
#include "state_warden.h"

namespace mediation_layer { 
  // The mediation layer is a software layer that mediates user input to ensure
  // that the trajectories provided to quadcopters are safe. During the
  // machine games, two user are allowed to specify trajectories for
  // quadcopters. The user-provided trajectories may not be safe --- the
  // trajetories might cause quads to fly into each other, walls, or other
  // obstacles in the environment. The Mediation
  template <size_t T>
  class MediationLayer {
    private:
      volatile std::atomic_bool ok_{true};

    public:
      MediationLayer() {}

      // Run the mediation layer.
      //
      // Note: These values are intentionally copied
      void Run(
          std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_in,
          std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_out,
          std::shared_ptr<StateWarden<T>> state_warden);

      void Stop();

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template<size_t T>
  inline void MediationLayer<T>::Run(
      std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_out,
      std::shared_ptr<StateWarden<T>> state_warden) {

    while(this->ok_) {
      const std::string key = "phoenix";
      Trajectory<T> trajectory;
      trajectory_warden_in->Await(key, trajectory);
      trajectory_warden_out->Write(key, trajectory);
    }
  }

  template<size_t T>
  inline void MediationLayer<T>::Stop() {
    this->ok_ = false;
  }

  using MediationLayer2D = MediationLayer<2>;
  using MediationLayer3D = MediationLayer<3>;
}
