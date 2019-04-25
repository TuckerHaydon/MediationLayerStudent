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

namespace game_engine {
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

      // Break all condition variable wait statements
      void Stop();
  };
}
