// Author: Tucker Haydon

#include "quad_state_watchdog_status.h"

namespace game_engine {
  void QuadStateWatchdogStatus::Register(const std::string& quad_name) {
    std::lock_guard<std::mutex> lock(mtx_);
    this->infraction_map_[quad_name] = false;
  }
  
  bool QuadStateWatchdogStatus::Read(const std::string& quad_name) const {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
      return this->infraction_map_.at(quad_name);
    } catch(std::out_of_range e) {
      std::cerr << "Quad with name " << quad_name 
        << " is not registered with the QuadStateWatchdog." << std::endl;
      return false;
    }
  }
  
  void QuadStateWatchdogStatus::Write(const std::string& quad_name, const bool infraction_occurred) {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
      this->infraction_map_.at(quad_name) = infraction_occurred;
    } catch(std::out_of_range e) {
      std::cerr << "Quad with name " << quad_name 
        << " is not registered with the QuadStateWatchdog." << std::endl;
      return;
    }
  }
}
