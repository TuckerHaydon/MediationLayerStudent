// Author: Tucker Haydon

#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>
#include <iostream>
#include <condition_variable>
#include <atomic>

#include "trajectory.h"

namespace mediation_layer {
  // TrajectoryWarden is a thread-safe abstraction around an unordered map
  // [trajectory_name -> trajectory]. TrajectoryWarden provides thread-safe access,
  // modification, and await-modification of the underlying trajectory.
  class TrajectoryWarden {
    private:
      // Wraps a Trajectory with local mutexes and condition variables that
      // ensure thread-safe access
      struct TrajectoryContainer {
        Trajectory trajectory_;
        std::mutex access_mtx_;

        std::mutex modified_mtx_;
        bool modified_{false};
        std::condition_variable modified_cv_;

        TrajectoryContainer(const Trajectory& trajectory)
          : trajectory_(trajectory) {}
      };

      std::unordered_map<std::string, std::shared_ptr<TrajectoryContainer>> map_;
      std::set<std::string> keys_;
      volatile std::atomic<bool> ok_{true};

    public:
      // Constructor
      TrajectoryWarden(){};

      // Add a key-value pair to the map
      bool Register(const std::string& key);

      // Write a new trajectory
      bool Write(const std::string& key,  const Trajectory& trajectory);

      // Copy the latest trajectory associated with a key
      bool Read(const std::string& key, Trajectory& trajectory);

      // Await a change to the trajectory associated with the key
      bool Await(const std::string& key, Trajectory& trajectory);

      // Getter
      const std::set<std::string>& Keys() const;

      void Stop();
  };
}

