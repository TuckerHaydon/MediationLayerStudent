// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <functional>

#include "state_warden.h"
#include "quad_state.h"

namespace mediation_layer {
  // StateSubscriberNode acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms incoming ROS data into a
  // QuadState and then passes it to the StateWarden to manage.
  template <size_t T>
  class StateSubscriberNode {
    private:
      std::shared_ptr<StateWarden<T>> warden_;
      ros::NodeHandle node_handle_;
      ros::Subscriber subscriber_;
      std::string key_;

      void SubscriberCallback(const std_msgs::String& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      StateSubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<StateWarden<T>> warden);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  StateSubscriberNode<T>::StateSubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<StateWarden<T>> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = ros::NodeHandle("~");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &StateSubscriberNode<T>::SubscriberCallback, 
        this);
  }

  template <size_t T>
  void StateSubscriberNode<T>::SubscriberCallback(const std_msgs::String& msg) {
    QuadState<T> state;
    this->warden_->Write(this->key_, state);
  }

  using StateSubscriberNode2D = StateSubscriberNode<2>;
  using StateSubscriberNode3D = StateSubscriberNode<3>;
};
