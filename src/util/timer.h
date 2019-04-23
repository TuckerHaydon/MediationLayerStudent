// Author: Tucker Haydon

#pragma once

#include <string>
#include <iostream>
#include <chrono>

namespace mediation_layer {
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
      void Start() {
        this->start_time_ 
          = std::chrono::high_resolution_clock::now();
      }

      // Stop the clock and return the duration
      std::chrono::duration<double> Stop() {
        this->end_time_ = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time 
          = this->end_time_ - this->start_time_;
        return elapsed_time;
      }

      // Print the elapsed time and the message
      void Print() {
        std::chrono::duration<double> elapsed_time = this->end_time_ - this->start_time_;
        std::chrono::milliseconds elapsed_ms 
          = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
        std::cout << this->message_ << std::endl;
        std::cout << "Elapsed Time: " << elapsed_ms.count() << " ms" << std::endl << std::endl;
      }
  };
}
