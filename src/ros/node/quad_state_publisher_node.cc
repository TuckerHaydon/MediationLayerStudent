// Author: Tucker Haydon

#include "quad_state_publisher_node.h"

namespace mediation_layer {
  QuadStatePublisherNode::QuadStatePublisherNode(
      const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<nav_msgs::Odometry>>(topic);
  }

  void QuadStatePublisherNode::Publish(const QuadState& quad_state) {
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
