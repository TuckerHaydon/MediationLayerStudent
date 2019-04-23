// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state_warden.h"
#include "quad_state.h"

#include "nav_msgs/Odometry.h"

namespace mediation_layer {
  // QuadStateSubscriberNode acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms incoming ROS data into a
  // QuadState and then passes it to the QuadStateWarden to manage.
  class QuadStateSubscriberNode {
    private:
      // Quad state warden that manages multi-threaded access to QuadState data
      std::shared_ptr<QuadStateWarden> warden_;

      // ROS node handle
      ros::NodeHandle node_handle_;

      // ROS subscriber
      ros::Subscriber subscriber_;

      // Key to use to pass on to the QuadStateWarden
      std::string key_;

      // Subscriber callback function. Extracts ROS data and converts it into a
      // QuadState to be passed on to the QuadStateWarden
      void SubscriberCallback(const nav_msgs::Odometry& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      QuadStateSubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<QuadStateWarden> warden);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline QuadStateSubscriberNode::QuadStateSubscriberNode(
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

  inline void QuadStateSubscriberNode::SubscriberCallback(const nav_msgs::Odometry& msg) {
    QuadState state(Eigen::Vector<double, 13>(
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
