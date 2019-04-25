// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state_warden.h"
#include "quad_state.h"

#include "nav_msgs/Odometry.h"

namespace game_engine {
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
}
