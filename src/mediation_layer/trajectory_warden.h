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

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool TrajectoryWarden::Register(const std::string& key) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      std::cerr << "TrajectoryWarden::Register -- Key already exists." << std::endl;
      return false;
    }

    this->map_[key] = std::make_shared<TrajectoryContainer>(Trajectory()); 
    keys_.insert(key);
    return true;
  }

  inline bool TrajectoryWarden::Write(const std::string& key, const Trajectory& trajectory) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "TrajectoryWarden::Write -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<TrajectoryContainer>& container = this->map_[key];
    
    { // Lock mutex for modification
      std::lock_guard<std::mutex> lock(container->access_mtx_);
      container->trajectory_ = trajectory;
    }

    { // Lock mutex for modification
      std::lock_guard<std::mutex> lock(container->modified_mtx_);
      container->modified_ = true;
      container->modified_cv_.notify_all();
    }
    return true;
  }

  inline bool TrajectoryWarden::Read(const std::string& key, Trajectory& trajectory) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "TrajectoryWarden::Read -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

    { // Lock mutex for copy
      std::lock_guard<std::mutex> lock(container->access_mtx_);
      trajectory = container->trajectory_;
    }
    return true;
  }
  
  inline bool TrajectoryWarden::Await(const std::string& key, Trajectory& trajectory) {
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "TrajectoryWarden::Await -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

    { // Lock mutex, wait cv, copy, set cv, release mutex
      std::unique_lock<std::mutex> lock(container->modified_mtx_);
      container->modified_cv_.wait(lock, [&]{  
          return (true == container->modified_) || (false == this->ok_); });
      { // Lock mutex for copy
        // Termination
        if(false == this->ok_) {
          return false;
        }

        std::lock_guard<std::mutex> lock(container->access_mtx_);
        trajectory = container->trajectory_;
      }
      container->modified_ = false;
      lock.unlock();
    }
    return true;
  }
  
  inline const std::set<std::string>& TrajectoryWarden::Keys() const {
    return this->keys_;
  }

  inline void TrajectoryWarden::Stop() {
    this->ok_ = false;

    // Notify all CV to check conditions
    for(const auto& kv: this->map_) {
      std::lock_guard<std::mutex> lock(kv.second->modified_mtx_);
      kv.second->modified_cv_.notify_all();
    }
  }
};

