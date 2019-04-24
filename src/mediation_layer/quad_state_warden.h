// Author: Tucker Haydon

#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>
#include <condition_variable>
#include <atomic>

#include "quad_state.h"

namespace mediation_layer {
  // QuadStateWarden encapsulates state data and provides thread-safe read, write,
  // and await-modification access.
  class QuadStateWarden {
    private:
      // Wraps a QuadState with local mutexes and condition variables that
      // ensure thread-safe access
      struct StateContainer {
        QuadState state_;
        std::mutex access_mtx_;

        std::mutex modified_mtx_;
        bool modified_{false};
        std::condition_variable modified_cv_;

        StateContainer(const QuadState& state)
          : state_(state) {}
      };

      std::unordered_map<std::string, std::shared_ptr<StateContainer>> map_;
      std::set<std::string> keys_;
      volatile std::atomic<bool> ok_{true};

    public:
      // Constructor
      QuadStateWarden(){};

      // Add a key-value pair to the map
      bool Register(const std::string& key);

      // Write a QuadState
      bool Write(const std::string& key,  const QuadState& state);

      // Copy the latest QuadState associated with a key
      bool Read(const std::string& key, QuadState& state);

      // Await a change to the state associated with the key
      bool Await(const std::string& key, QuadState& state);

      // Getter
      const std::set<std::string>& Keys() const;

      void Stop();

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool QuadStateWarden::Register(const std::string& key) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      std::cerr << "QuadStateWarden::Register -- Key already exists." << std::endl;
      return false;
    }

    this->map_[key] = std::make_shared<StateContainer>(QuadState()); 
    keys_.insert(key);
    return true;
  }

  inline bool QuadStateWarden::Write(const std::string& key, const QuadState& state) {
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

  inline bool QuadStateWarden::Read(const std::string& key, QuadState& state) {
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
  
  inline bool QuadStateWarden::Await(const std::string& key, QuadState& state) {
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
  
  inline const std::set<std::string>& QuadStateWarden::Keys() const {
    return this->keys_;
  }

  inline void QuadStateWarden::Stop() {
    this->ok_ = false;

    // Notify all CV to check conditions
    for(const auto& kv: this->map_) {
      std::lock_guard<std::mutex> lock(kv.second->modified_mtx_);
      kv.second->modified_cv_.notify_all();
    }
  }
}