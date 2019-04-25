// Author: Tucker Haydon

#include <iostream>

#include "quad_state_warden.h"

namespace game_engine {
  bool QuadStateWarden::Register(const std::string& key) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      std::cerr << "QuadStateWarden::Register -- Key already exists." << std::endl;
      return false;
    }

    this->map_[key] = std::make_shared<StateContainer>(QuadState()); 
    keys_.insert(key);
    return true;
  }

  bool QuadStateWarden::Write(const std::string& key, const QuadState& state) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "QuadStateWarden::Write -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<StateContainer>& container = this->map_[key];
    
    { // Lock mutex for modification
      std::lock_guard<std::mutex> lock(container->access_mtx_);
      container->state_ = state;
    }

    { // Lock mutex for modification
      std::lock_guard<std::mutex> lock(container->modified_mtx_);
      container->modified_ = true;
      container->modified_cv_.notify_all();
    }
    return true;
  }

  bool QuadStateWarden::Read(const std::string& key, QuadState& state) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "QuadStateWarden::Read -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<StateContainer>& container = this->map_[key];

    { // Lock mutex for copy
      std::lock_guard<std::mutex> lock(container->access_mtx_);
      state = container->state_;
    }
    return true;
  }
  
  bool QuadStateWarden::Await(const std::string& key, QuadState& state) {
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "QuadStateWarden::Await -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<StateContainer>& container = this->map_[key];

    { // Lock mutex, wait cv, copy, set cv, release mutex
      std::unique_lock<std::mutex> lock(container->modified_mtx_);
      container->modified_cv_.wait(lock, [&]{ return (true == container->modified_) || (false == this->ok_); });
      { // Lock mutex for copy
        // Termination
        if(false == this->ok_) {
          return false;
        }
        std::lock_guard<std::mutex> lock(container->access_mtx_);
        state = container->state_;
      }
      container->modified_ = false;
      lock.unlock();
    }
    return true;
  }
  
  const std::set<std::string>& QuadStateWarden::Keys() const {
    return this->keys_;
  }

  void QuadStateWarden::Stop() {
    this->ok_ = false;

    // Notify all CV to check conditions
    for(const auto& kv: this->map_) {
      std::lock_guard<std::mutex> lock(kv.second->modified_mtx_);
      kv.second->modified_cv_.notify_all();
    }
  }
}

