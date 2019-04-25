// Author: Tucker Haydon

#include <iostream>

#include "timer.h"

namespace game_engine {
  void Timer::Start() {
    this->start_time_ 
      = std::chrono::high_resolution_clock::now();
  }

  std::chrono::duration<double> Timer::Stop() {
    this->end_time_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time 
      = this->end_time_ - this->start_time_;
    return elapsed_time;
  }

  void Timer::Print() {
    std::chrono::duration<double> elapsed_time = this->end_time_ - this->start_time_;
    std::chrono::milliseconds elapsed_ms 
      = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
    std::cout << this->message_ << std::endl;
    std::cout << "Elapsed Time: " << elapsed_ms.count() << " ms" << std::endl << std::endl;
  }
}
