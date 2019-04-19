// Author: Tucker Haydon

#pragma once

#include <thread>
#include <atomic>

namespace mediation_layer {
  class Thread {
    protected: 
      std::thread thread_;
      volatile std::atomic<bool> ok_;

    public:
      void Start();
      void Stop();
      void Join();

      virtual void Run() = 0;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline void Thread::Start() {
    this->thread_ = std::thread(
        [&]() {
          while(true == this->ok_) {
            this->Run();
          }
        });
  }

  inline void Thread::Stop() {
    this->ok_ = false;
  }

  inline void Thread::Join() {
    this->thread_.join();
  }
}
