// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state_warden.h"
#include "quad_state.h"

namespace mediation_layer {
  // QuadStateSubscriberNode acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms incoming ROS data into a
  // QuadState and then passes it to the QuadStateWarden to manage.
  template <size_t T>
  class QuadStateSubscriberNode {
    private:
      std::shared_ptr<QuadStateWarden<T>> warden_;
      ros::NodeHandle node_handle_;
      ros::Subscriber subscriber_;
      std::string key_;

      void SubscriberCallback(const std_msgs::String& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      QuadStateSubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<QuadStateWarden<T>> warden);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  QuadStateSubscriberNode<T>::QuadStateSubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<QuadStateWarden<T>> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = ros::NodeHandle("~");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &QuadStateSubscriberNode<T>::SubscriberCallback, 
        this);
  }

  template <size_t T>
  void QuadStateSubscriberNode<T>::SubscriberCallback(const std_msgs::String& msg) {
    QuadState<T> state;
    this->warden_->Write(this->key_, state);
  }

  using QuadStateSubscriberNode2D = QuadStateSubscriberNode<2>;
  using QuadStateSubscriberNode3D = QuadStateSubscriberNode<3>;
};
