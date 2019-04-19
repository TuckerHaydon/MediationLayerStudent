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
  class QuadStatePublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<nav_msgs::Odometry>> publisher_guard_;
      
    public:
      // Constructor.
      QuadStatePublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const QuadState& quad_state);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline QuadStatePublisherNode::QuadStatePublisherNode(
      const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<nav_msgs::Odometry>>(topic);
  }

  inline void QuadStatePublisherNode::Publish(const QuadState& quad_state) {
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.pose.pose.position.x = quad_state.Position().x();
    msg.pose.pose.position.y = quad_state.Position().y();
    msg.pose.pose.position.z = quad_state.Position().z();
    msg.twist.twist.linear.x = quad_state.Velocity().x();
    msg.twist.twist.linear.y = quad_state.Velocity().y();
    msg.twist.twist.linear.z = quad_state.Velocity().z();
    msg.pose.pose.orientation.w = 1;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;
    msg.twist.twist.angular.x = quad_state.Twist().x();
    msg.twist.twist.angular.y = quad_state.Twist().z();
    msg.twist.twist.angular.z = quad_state.Twist().y();
    this->publisher_guard_->Publish(msg);
  }
}
