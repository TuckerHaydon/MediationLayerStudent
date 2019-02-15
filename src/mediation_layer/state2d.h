// Author: Tucker Haydon

#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>

#include "trajectory2d.h"

namespace mediation_layer {
  class State2D {
    private:
      struct TrajectoryContainer {
        Trajectory2D trajectory_;
        std::mutex mtx_;

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
    
    { // Lock mutex for copy
      std::shared_ptr<TrajectoryContainer> container = this->map_[key];
      std::lock_guard<std::mutex> lock(container->mtx_);
      container->trajectory_ = trajectory;
    }
    return true;
  }

  inline bool State2D::Read(const std::string& key, Trajectory2D& trajectory) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      return false;
    }

    { // Lock mutex for copy
      std::shared_ptr<TrajectoryContainer> container = this->map_[key];
      std::lock_guard<std::mutex> lock(container->mtx_);
      trajectory = container->trajectory_;
    }
    return true;
  }

};

