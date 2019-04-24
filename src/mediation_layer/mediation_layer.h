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

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline void MediationLayer::TransferData(
      const std::string& key,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out) {

      while(true == this->ok_) {
        Trajectory trajectory;
        trajectory_warden_in->Await(key, trajectory);
        trajectory_warden_out->Write(key, trajectory);
      }
  }

  inline void MediationLayer::Run(
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
      std::shared_ptr<QuadStateWarden> state_warden) {

    // Local thread pool 
    std::vector<std::thread> thread_pool;

    // Get all registered trajectories
    const std::set<std::string> state_keys = state_warden->Keys();

    // Assign a thread to await changes in each trajectory
    for(const std::string& key: state_keys) {
      thread_pool.push_back(
          std::move(
            std::thread([&](){
              this->TransferData(key, trajectory_warden_in, trajectory_warden_out);
              })));
    }

    // Wait for this thread to receive a stop command
    std::thread kill_thread(
        [&, this]() {
          while(true) {
            if(false == this->ok_) {
              break;
            } else {
              std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
          }
        });

    kill_thread.join();

    // Wait for thread pool to terminate
    for(std::thread& t: thread_pool) {
      t.join();
    } 
  }

  inline void MediationLayer::Stop() {
    this->ok_ = false;
  }
}
