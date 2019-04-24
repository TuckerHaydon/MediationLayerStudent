// Author: Tucker Haydon

#include <thread>

#include "mediation_layer.h"

namespace mediation_layer {
  void MediationLayer::TransferData(
      const std::string& key,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out) {

      while(true == this->ok_) {
        Trajectory trajectory;
        trajectory_warden_in->Await(key, trajectory);
        trajectory_warden_out->Write(key, trajectory);
      }
  }

  void MediationLayer::Run(
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

  void MediationLayer::Stop() {
    this->ok_ = false;
  }
}

