// Author: Tucker Haydon

#include "quad_state_subscriber_node.h"

namespace mediation_layer {
  QuadStateSubscriberNode::QuadStateSubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<QuadStateWarden> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = ros::NodeHandle("/mediation_layer/");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &QuadStateSubscriberNode::SubscriberCallback, 
        this);
  }

  void QuadStateSubscriberNode::SubscriberCallback(const nav_msgs::Odometry& msg) {
    QuadState state(Eigen::Matrix<double, 13, 1>(
          msg.pose.pose.position.x, 
          msg.pose.pose.position.y, 
          msg.pose.pose.position.z, 
          msg.twist.twist.linear.x,
          msg.twist.twist.linear.y,
          msg.twist.twist.linear.z,
          msg.pose.pose.orientation.w,
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.twist.twist.angular.x,
          msg.twist.twist.angular.y,
          msg.twist.twist.angular.z
          ));
    this->warden_->Write(this->key_, state);
  }
}
