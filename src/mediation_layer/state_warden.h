// Author: Tucker Haydon

#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>
#include <condition_variable>

#include "quad_state.h"

namespace mediation_layer {
  // StateWarden encapsulates state data and provides thread-safe read, write,
  // and await-modification access.
  template <size_t T>
  class StateWarden {
    private:
      // Wraps a QuadState with local mutexes and condition variables that
      // ensure thread-safe access
      struct StateContainer {
        QuadState<T> state_;
        std::mutex access_mtx_;

        std::mutex modified_mtx_;
        bool modified_{false};
        std::condition_variable modified_cv_;

        StateContainer(const QuadState<T>& state)
          : state_(state) {}
      };

      std::unordered_map<std::string, std::shared_ptr<StateContainer>> map_;
      std::set<std::string> keys_;

    public:
      // Constructor
      StateWarden(){};

      // Add a key-value pair to the map
      bool Register(const std::string& key);

      // Write a QuadState
      bool Write(const std::string& key,  const QuadState<T>& state);

      // Copy the latest QuadState associated with a key
      bool Read(const std::string& key, QuadState<T>& state);

      // Await a change to the state associated with the key
      bool Await(const std::string& key, QuadState<T>& state);

      // Getter
      const std::set<std::string>& Keys() const;

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline bool StateWarden<T>::Register(const std::string& key) {
    // If this key already exists, return false
    if(this->map_.end() != this->map_.find(key)) {
      std::cerr << "StateWarden::Register -- Key already exists." << std::endl;
      return false;
    }

    this->map_[key] = std::make_shared<StateContainer>(QuadState<T>()); 
    keys_.insert(key);
    return true;
  }

  template <size_t T>
  inline bool StateWarden<T>::Write(const std::string& key, const QuadState<T>& state) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "StateWarden::Write -- Key does not exist." << std::endl;
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

  template <size_t T>
  inline bool StateWarden<T>::Read(const std::string& key, QuadState<T>& state) {
    // If key does not exist, return false
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "StateWarden::Read -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<StateContainer>& container = this->map_[key];

    { // Lock mutex for copy
      std::lock_guard<std::mutex> lock(container->access_mtx_);
      state = container->state_;
    }
    return true;
  }
  
  template <size_t T>
  inline bool StateWarden<T>::Await(const std::string& key, QuadState<T>& state) {
    if(this->map_.end() == this->map_.find(key)) {
      std::cerr << "StateWarden::Await -- Key does not exist." << std::endl;
      return false;
    }
    std::shared_ptr<StateContainer>& container = this->map_[key];

    { // Lock mutex, wait cv, copy, set cv, release mutex
      std::unique_lock<std::mutex> lock(container->modified_mtx_);
      container->modified_cv_.wait(lock, [container]{ return true == container->modified_; });
      { // Lock mutex for copy
        std::lock_guard<std::mutex> lock(container->access_mtx_);
        state = container->state_;
      }
      container->modified_ = false;
      lock.unlock();
    }
    return true;
  }
  
  template <size_t T>
  inline const std::set<std::string>& StateWarden<T>::Keys() const {
    return this->keys_;
  }

  using StateWarden2D = StateWarden<2>;
  using StateWarden3D = StateWarden<3>;
}
