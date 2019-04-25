// Author: Tucker Haydon

#pragma once

#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>

#include "trajectory_warden.h"
#include "game_snapshot.h"
#include "map3d.h"

namespace game_engine {
  // The AutonomyProtocol interfaces with the GameSimulator and enables an
  // actor to read limited information from the GameSimulator and report
  // intended future actions for specific quadcopters.
  //
  // This class is an interface --- it should not be constructed as a standalone
  // object. Instead, a sub-class should implement this class. Sub-classes must
  // implement the UpdateTrajectories function. The UpdateTrajectories function
  // should map friendly quadcopters current positions to intended trajectories.
  //
  // UpdateTrajectories is called every 200ms (5 hz). A protocol may either
  // specify a new trajectory, overwriting the last trajectory, or return an
  // empty trajectory, implicitly instructing the quadcopter to continue following 
  // the previous trajectory or holding the final position. 
  //
  // Warning: sometimes ROS drops the first couple of messages in a stream. It's
  // busy connecting publishers and subscribers together and does not properly
  // pass on messages. As a result, if a trajectory is only published the first
  // time there is no guarantee that it gets to the destination. The trajectory
  // may need to be re-published.
  //
  // The AutonomyProtocol should be run as its own thread.
  class AutonomyProtocol {
    protected:
      std::vector<std::string> friendly_names_;
      std::vector<std::string> enemy_names_;
      std::shared_ptr<GameSnapshot> snapshot_;
      std::shared_ptr<TrajectoryWarden> trajectory_warden_out_;
      Map3D map3d_;
      std::map<
        std::string, 
        Eigen::Vector3d, 
        std::less<std::string>, 
        Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> balloon_map_;
      volatile std::atomic<bool> ok_{true};

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      AutonomyProtocol(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot> snapshot,
          const std::shared_ptr<TrajectoryWarden> trajectory_warden_out,
          const Map3D& map3d,
          std::map<
            std::string, 
            Eigen::Vector3d, 
            std::less<std::string>, 
            Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> balloon_map)
        : friendly_names_(friendly_names),
          enemy_names_(enemy_names),
          snapshot_(snapshot),
          trajectory_warden_out_(trajectory_warden_out),
          map3d_(map3d),
          balloon_map_(balloon_map) {};

      virtual ~AutonomyProtocol(){}

      // Stop this thread from running
      void Stop();

      // Main loop for this thread
      void Run();

      // Virtual function to be implemented as by an actor. Input is a snapshot
      // of the system, output is an intended trajectory for each of the
      // friendly quads.
      virtual std::unordered_map<std::string, Trajectory> UpdateTrajectories() = 0;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline void AutonomyProtocol::Run() {
    while(this->ok_) {

      // Request trajectory updates from the virtual function
      const std::unordered_map<std::string, Trajectory> trajectories = 
        this->UpdateTrajectories();


      // For every friendly quad, push the intended trajectory to the trajectory
      // warden
      for(const std::string& quad_name: this->friendly_names_) {
        try {
          const Trajectory trajectory = trajectories.at(quad_name);
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

  inline void AutonomyProtocol::Stop() {
    this->ok_ = false;
  }
}
