// Author: Tucker Haydon

#pragma once

#include <string>
#include <chrono>

namespace game_engine {
  // Utility timer class
  class Timer {
    private:
      // Start and end times
      std::chrono::time_point<std::chrono::high_resolution_clock> 
        start_time_, end_time_;

      // Message to print when timer is done
      std::string message_;

    public:
      Timer(const std::string& message = "")
        : message_(message) {}

    // Start the clock
    void Start();
  
    // Stop the clock and return the duration
    std::chrono::duration<double> Stop();
  
    // Print the elapsed time and the message
    void Print();
  };
}
