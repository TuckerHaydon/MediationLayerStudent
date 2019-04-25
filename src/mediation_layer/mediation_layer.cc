// Author: Tucker Haydon

#include <thread>

#include "mediation_layer.h"
#include "trajectory_vetter.h"

namespace game_engine {
    void MediationLayer::TransferData(
       const std::string& key,
       const Map3D& map,
       std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
       std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
       std::shared_ptr<QuadStateWarden> quad_state_warden) {

      TrajectoryVetter trajectory_vetter;
      while(true == this->ok_) {
        Trajectory trajectory;
        trajectory_warden_in->Await(key, trajectory);

        if(false == trajectory_vetter.Vet(
              trajectory,
              map,
              quad_state_warden,
              key)) {
          std::cerr << "Trajectory did not pass vetting. Rejected." << std::endl;
          continue;
        }

        trajectory_warden_out->Write(key, trajectory);
      }
  }

  void MediationLayer::Run(
      const Map3D& map,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
      std::shared_ptr<QuadStateWarden> quad_state_warden) {

    // Local thread pool 
    std::vector<std::thread> thread_pool;

    // Get all registered trajectories
    const std::set<std::string> state_keys = quad_state_warden->Keys();

    // Assign a thread to await changes in each trajectory
    for(const std::string& key: state_keys) {
      thread_pool.push_back(
          std::move(
            std::thread([&](){
              this->TransferData(key, map, trajectory_warden_in, trajectory_warden_out, quad_state_warden);
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

