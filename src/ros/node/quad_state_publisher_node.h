// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state.h"
#include "publisher_guard.h"
#include "nav_msgs/Odometry.h"

namespace mediation_layer {
  // QuadStatePublisherNode acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms internal QuadState into a
  // ROS quad_state and publishes it.
  template <size_t T>
  class QuadStatePublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<nav_msgs::Odometry>> publisher_guard_;
      
    public:
      // Constructor.
      QuadStatePublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const QuadState<T>& quad_state);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline QuadStatePublisherNode<T>::QuadStatePublisherNode(
      const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<nav_msgs::Odometry>>(topic);
  }

  template <size_t T>
  inline void QuadStatePublisherNode<T>::Publish(const QuadState<T>& quad_state) {
    nav_msgs::Odometry msg;
    this->publisher_guard_->Publish(msg);
  }

  using QuadStatePublisherNode2D = QuadStatePublisherNode<2>;
  using QuadStatePublisherNode3D = QuadStatePublisherNode<3>;
}
