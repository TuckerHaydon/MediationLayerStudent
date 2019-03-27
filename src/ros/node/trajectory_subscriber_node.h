// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory_warden.h"
#include "trajectory.h"

namespace mediation_layer {
  template <size_t T>
  class TrajectorySubscriberNode {
    private:
      std::shared_ptr<TrajectoryWarden<T>> warden_;
      std::shared_ptr<ros::NodeHandle> node_handle_;
      std::shared_ptr<ros::Subscriber> subscriber_;
      std::string key_;

      void SubscriberCallback(const std_msgs::String::ConstPtr& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      TrajectorySubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<TrajectoryWarden<T>> warden);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  TrajectorySubscriberNode<T>::TrajectorySubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<TrajectoryWarden<T>> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = std::make_shared<ros::NodeHandle>("~");
    this->subscriber_ = node_handle_->subscribe(
        topic, 
        1, 
        std::bind(
          &TrajectorySubscriberNode<T>::SubscriberCallback, 
          this, 
          std::placeholders::_1));
  }

  template <size_t T>
  TrajectorySubscriberNode<T>::SubscriberCallback(const std_msgs::String::ConstPtr& msg) {
    Trajectory<T> trajectory;
    this->warden->Write(this->key_, trajectory);
  }
};
