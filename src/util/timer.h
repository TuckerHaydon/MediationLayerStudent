// Author: Tucker Haydon

#ifndef PATH_PLANNING_UTIL_TIMER
#define PATH_PLANNING_UTIL_TIMER

#include <string>
#include <iostream>
#include <chrono>

namespace path_planning {

  class Timer {
    private:
      typedef std::chrono::high_resolution_clock Time;
      typedef std::chrono::milliseconds ms;
      typedef std::chrono::duration<double> duration_t;

      std::chrono::time_point<Time> start_time_, end_time_;
      std::string message_;

    public:
      Timer(const std::string& message);
      bool Start();
      bool Stop();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Timer::Timer(const std::string& message) {
    this->message_ = message;
  }

  inline bool Timer::Start() {
    this->start_time_ = Time::now();
    return true;
  }

  inline bool Timer::Stop() {
    this->end_time_ = Time::now();
    duration_t elapsed_time = this->end_time_ - this->start_time_;
    ms elapsed_ms = std::chrono::duration_cast<ms>(elapsed_time);
    std::cout << this->message_ << std::endl;
    std::cout << "Elapsed Time: " << elapsed_ms.count() << " ms" << std::endl << std::endl;
    return true;
  }
};

#endif
