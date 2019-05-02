// Author: Tucker Haydon

#include <thread>
#include <chrono>

#include "mediation_layer.h"
#include "trajectory_vetter.h"

namespace game_engine {
    void MediationLayer::TransferData(
       const std::string& key,
       const Map3D& map,
       std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
       std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
       std::shared_ptr<QuadStateWarden> quad_state_warden,
       std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status) {

      TrajectoryVetter trajectory_vetter;
      while(true == this->ok_) {
        Trajectory trajectory;
        trajectory_warden_in->Await(key, trajectory);

        // Determine if trajectory has violated constraints
        if(false == trajectory_vetter.Vet(
              trajectory,
              map,
              quad_state_warden,
              key)) {
          std::cerr << "Trajectory did not pass vetting. Rejected." << std::endl;
          continue;
        }

        // Determine if quad has violated state constraints. If it has, freeze
        // it in place
        if(true == quad_state_watchdog_status->Read(key)) {
          std::cout << key << " has flown too close to obstacle of boundary. Freezing." << std::endl;

          QuadState current_quad_state;
          quad_state_warden->Read(key, current_quad_state);

          const Eigen::Vector3d current_quad_position = current_quad_state.Position();

          const double current_time = 
            std::chrono::duration_cast<std::chrono::duration<double>>(
              std::chrono::system_clock::now().time_since_epoch()).count();

          TrajectoryVector3D freeze_trajectory_vector;
          freeze_trajectory_vector.push_back(Eigen::Matrix<double, 11, 1>(
                current_quad_position.x(), current_quad_position.y(), current_quad_position.z(),
                0,0,0,
                0,0,0,
                0,
                current_time
                )
            );
          const Trajectory freeze_trajectory(freeze_trajectory_vector);

          trajectory_warden_out->Write(key, freeze_trajectory_vector);
          continue;
        }

        trajectory_warden_out->Write(key, trajectory);
      }
  }

  void MediationLayer::Run(
      const Map3D& map,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status) {

    // Local thread pool 
    std::vector<std::thread> thread_pool;

    // Get all registered trajectories
    const std::set<std::string> state_keys = quad_state_warden->Keys();

    // Assign a thread to await changes in each trajectory
    for(const std::string& key: state_keys) {
      thread_pool.push_back(
          std::move(
            std::thread([&](){
              this->TransferData(key, map, trajectory_warden_in, trajectory_warden_out, quad_state_warden, quad_state_watchdog_status);
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

