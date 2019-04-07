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
  template <size_t T>
  class TrajectoryWarden {
    private:
      // Wraps a Trajectory with local mutexes and condition variables that
      // ensure thread-safe access
      struct TrajectoryContainer {
        Trajectory<T> trajectory_;
        std::mutex access_mtx_;

        std::mutex modified_mtx_;
        bool modified_{false};
        std::condition_variable modified_cv_;

        TrajectoryContainer(const Trajectory<T>& trajectory)
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
      bool Write(const std::string& key,  const Trajectory<T>& trajectory);

      // Copy the latest trajectory associated with a key
      bool Read(const std::string& key, Trajectory<T>& trajectory);

      // Await a change to the trajectory associated with the key
      bool Await(const std::string& key, Trajectory<T>& trajectory);

      // Getter
      const std::set<std::string>& Keys() const;

      void Stop();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline bool TrajectoryWarden<T>::Register(const std::string& key) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      std::cerr << "TrajectoryWarden::Register -- Key already exists." << std::endl;
      return false;
    }

    this->map_[key] = std::make_shared<TrajectoryContainer>(Trajectory<T>()); 
    keys_.insert(key);
    return true;
  }

  template <size_t T>
  inline bool TrajectoryWarden<T>::Write(const std::string& key, const Trajectory<T>& trajectory) {
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

  template <size_t T>
  inline bool TrajectoryWarden<T>::Read(const std::string& key, Trajectory<T>& trajectory) {
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
  
  template <size_t T>
  inline bool TrajectoryWarden<T>::Await(const std::string& key, Trajectory<T>& trajectory) {
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
  
  template <size_t T>
  inline const std::set<std::string>& TrajectoryWarden<T>::Keys() const {
    return this->keys_;
  }

  template <size_t T>
  inline void TrajectoryWarden<T>::Stop() {
    this->ok_ = false;

    // Notify all CV to check conditions
    for(const auto& kv: this->map_) {
      std::lock_guard<std::mutex> lock(kv.second->modified_mtx_);
      kv.second->modified_cv_.notify_all();
    }
  }

  using TrajectoryWarden2D = TrajectoryWarden<2>;
  using TrajectoryWarden3D = TrajectoryWarden<3>;

};

