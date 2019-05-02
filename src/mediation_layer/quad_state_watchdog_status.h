// Author: Tucker Haydon

#pragma once

#include <atomic>
#include <string>
#include <mutex>
#include <unordered_map>
#include <stdexcept>
#include <iostream>

namespace game_engine {
  // Struct that contains information about the status of the QuadStateWatchdog. If
  // the QuadStateWatchdog determines that a quadcopter has flown too close to an
  // obstacle, an instance of this status is updated to reflect that.
  class QuadStateWatchdogStatus {
    private:
      // Mutex to manage multi-threaded access
      mutable std::mutex mtx_;

      // Map from quad name to whether or not an infraction has occurred
      std::unordered_map<std::string, bool> infraction_map_;

    public:
      QuadStateWatchdogStatus() {}
      void Register(const std::string& quad_name);
      bool Read(const std::string& quad_name) const;
      void Write(const std::string& quad_name, const bool infraction_occurred);
  };
}
