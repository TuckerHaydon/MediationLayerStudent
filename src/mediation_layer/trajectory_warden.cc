// Author: Tucker Haydon

#include "trajectory_warden.h"

namespace mediation_layer {
  bool TrajectoryWarden::Register(const std::string& key) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      std::cerr << "TrajectoryWarden::Register -- Key already exists." << std::endl;
      return false;
    }

    this->map_[key] = std::make_shared<TrajectoryContainer>(Trajectory()); 
    keys_.insert(key);
    return true;
  }

  bool TrajectoryWarden::Write(const std::string& key, const Trajectory& trajectory) {
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

  bool TrajectoryWarden::Read(const std::string& key, Trajectory& trajectory) {
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
  
  bool TrajectoryWarden::Await(const std::string& key, Trajectory& trajectory) {
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
  
  const std::set<std::string>& TrajectoryWarden::Keys() const {
    return this->keys_;
  }

  void TrajectoryWarden::Stop() {
    this->ok_ = false;

    // Notify all CV to check conditions
    for(const auto& kv: this->map_) {
      std::lock_guard<std::mutex> lock(kv.second->modified_mtx_);
      kv.second->modified_cv_.notify_all();
    }
  }
}

