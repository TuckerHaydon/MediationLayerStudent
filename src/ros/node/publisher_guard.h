// Author: Tucker Haydon

#pragma once

#include <memory>
#include <mutex>
#include <ros/ros.h>

namespace mediation_layer {
  // Publisher guard is a wrapper around a publisher that ensures thread-safe,
  // sychronized access to a publisher. 
  template <class T>
  class PublisherGuard {
    private:
      ros::NodeHandle node_handle_;
      ros::Publisher publisher_;
      std::mutex mtx_;

    public:
      PublisherGuard(const std::string& topic);

      bool Publish(const T& msg);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <class T>
  PublisherGuard<T>::PublisherGuard(const std::string& topic) {
    this->node_handle_ = ros::NodeHandle("~");
    this->publisher_ = this->node_handle_.advertise<T>(topic, 1);
  }

  template <class T>
  inline bool PublisherGuard<T>::Publish(const T& msg) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->publisher_.publish(msg);
    return true;
  }
}
