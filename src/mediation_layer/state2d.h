// Author: Tucker Haydon

#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>
#include <condition_variable>

#include "trajectory2d.h"

namespace mediation_layer {
  class State2D {
    private:
      struct TrajectoryContainer {
        Trajectory2D trajectory_;
        std::mutex access_mtx_;

        std::mutex modified_mtx_;
        bool modified_{false};
        std::condition_variable modified_cv_;

        TrajectoryContainer(const Trajectory2D& trajectory)
          : trajectory_(trajectory) {}
      };

      std::unordered_map<std::string, std::shared_ptr<TrajectoryContainer>> map_;
      std::set<std::string> keys_;

    public:
      State2D(){};
      bool Add(const std::string& key, const Trajectory2D& trajectory);
      bool Write(const std::string& key,  const Trajectory2D& trajectory);
      bool Read(const std::string& key, Trajectory2D& trajectory);
      bool Await(const std::string& key, Trajectory2D& trajectory);
      const std::set<std::string>& Keys() const;

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool State2D::Add(const std::string& key, const Trajectory2D& trajectory) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      return false;
    }

    this->map_[key] = std::make_shared<TrajectoryContainer>(trajectory); 
    keys_.insert(key);
    return true;
  }

  inline bool State2D::Write(const std::string& key, const Trajectory2D& trajectory) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
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
      container->modified_cv_.notify_one();
    }
    return true;
  }

  inline bool State2D::Read(const std::string& key, Trajectory2D& trajectory) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      return false;
    }
    std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

    { // Lock mutex for copy
      std::lock_guard<std::mutex> lock(container->access_mtx_);
      trajectory = container->trajectory_;
    }
    return true;
  }
  
  inline bool State2D::Await(const std::string& key, Trajectory2D& trajectory) {
    if(this->map_.end() == this->map_.find(key)) {
      return false;
    }
    std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

    { // Lock mutex, wait cv, copy, set cv, release mutex
      std::unique_lock<std::mutex> lock(container->modified_mtx_);
      container->modified_cv_.wait(lock, [container]{ return true == container->modified_; });
      trajectory = container->trajectory_;
      container->modified_ = false;
      lock.unlock();
    }
    return true;
  }
  
  inline const std::set<std::string>& State2D::Keys() const {
    return this->keys_;
  }

};

