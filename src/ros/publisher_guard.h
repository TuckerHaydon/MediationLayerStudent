// Author: Tucker Haydon

#pragma once

#include <memory>
#include <mutex>
#include <ros/ros.h>

namespace mediation_layer {
  // Publisher guard is a wrapper around a publisher that ensures thread-safe,
  // sychronized assess to a publisher. 
  template <class T>
  class PublisherGuard {
    private:
      std::shared_ptr<ros::Publisher> publisher_;
      std::mutex mtx_;

    public:
      PublisherGuard(const std::shared_ptr<ros::Publisher> publisher)
        : publisher_(publisher) {}

      bool Publish(const T& msg);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <class T>
  inline bool PublisherGuard<T>::Publish(const T& msg) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->publisher_.publish(msg);
    return true;
  }
}
