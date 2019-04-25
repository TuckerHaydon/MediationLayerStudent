// Author: Tucker Haydon

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <ros/ros.h>

namespace game_engine {
  // Publisher guard is a wrapper around a publisher that ensures thread-safe,
  // sychronized access to a publisher. 
  //
  // TODO: What should the parameter to NodeHandle be?
  template <class T>
  class PublisherGuard {
    private:
      // Ros node handle
      ros::NodeHandle node_handle_;

      // Ros publisher
      ros::Publisher publisher_;

      // Mutex that guards the publisher
      std::mutex mtx_;

    public:
      // Constructor
      PublisherGuard(const std::string& topic);

      // Publish the message after capturing the mutex
      void Publish(const T& msg);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <class T>
  inline PublisherGuard<T>::PublisherGuard(const std::string& topic) {
    this->node_handle_ = ros::NodeHandle("~");
    this->publisher_ = node_handle_.advertise<T>(topic, 1);
  }

  template <class T>
  inline void PublisherGuard<T>::Publish(const T& msg) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->publisher_.publish(msg);
  }
}
