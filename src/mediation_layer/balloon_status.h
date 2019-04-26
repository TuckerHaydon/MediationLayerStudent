// Author: Tucker Haydon

#pragma once

#include <string>
#include <chrono>

namespace game_engine {
  // Plain-old-data structure containing information regarding a balloon's
  // status
  struct BalloonStatus {
    // Whether the balloon has been popped or not
    bool popped = false;

    // Which quad popped the balloon
    std::string popper = "null";

    // The time the balloon was popped at
    std::chrono::system_clock::time_point pop_time;
  };
}
