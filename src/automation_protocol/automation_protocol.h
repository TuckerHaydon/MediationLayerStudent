// Author: Tucker Haydon

#pragma once

#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <map>
#include <string>

#include "trajectory_warden.h"
#include "game_snapshot.h"

namespace mediation_layer {
  // The AutomationProtocol interfaces with the GameSimulator and enables an
  // actor to read limited information from the GameSimulator and report
  // intended future actions for specific quadcopters.
  //
  // The AutomationProtocol should be run as its own thread.
  template <size_t T>
  class AutomationProtocol {
    protected:
      std::vector<std::string> friendly_names_;
      std::vector<std::string> enemy_names_;
      std::shared_ptr<GameSnapshot<T>> snapshot_;
      std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_out_;
      std::atomic<bool> ok_{true};

    public:
      AutomationProtocol(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot<T>> snapshot,
          const std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_out)
        : friendly_names_(friendly_names),
          enemy_names_(enemy_names),
          snapshot_(snapshot),
          trajectory_warden_out_(trajectory_warden_out) {}

      virtual ~AutomationProtocol(){}

      // Stop this thread from running
      void Stop();

      // Main loop for this thread
      void Run();

      // Virtual function to be implemented as by an actor. Input is a snapshot
      // of the system, output is an intended trajectory for each of the
      // friendly quads.
      virtual std::unordered_map<std::string, Trajectory<T>> UpdateTrajectories(
          std::shared_ptr<GameSnapshot<T>> snapshot,
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names) = 0;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  void AutomationProtocol<T>::Run() {
    while(this->ok_) {

      // Request trajectory updates from the virtual function
      const std::unordered_map<std::string, Trajectory<T>> trajectories = 
        this->UpdateTrajectories(this->snapshot_, this->friendly_names_, this->enemy_names_);


      // For every friendly quad, push the intended trajectory to the trajectory
      // warden
      for(const std::string& quad_name: this->friendly_names_) {
        try {
          const Trajectory<T> trajectory = trajectories.at(quad_name);
          this->trajectory_warden_out_->Write(quad_name, trajectory);
        } catch(const std::out_of_range& e) {
          continue;
        }
      }

      // Sleep for 200 ms. This essentially creates a loop that runs at 5 Hz. A
      // new trajectory may be specified once every 200 ms.
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  template <size_t T>
  void AutomationProtocol<T>::Stop() {
    this->ok_ = false;
  }

  using AutomationProtocol2D = AutomationProtocol<2>;
  using AutomationProtocol3D = AutomationProtocol<3>;
}
