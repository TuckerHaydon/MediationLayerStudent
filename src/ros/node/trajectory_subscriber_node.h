// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory_warden.h"
#include "trajectory.h"
#include "mg_msgs/PVAYStampedTrajectory.h"

namespace game_engine {
  // Trajectory subscriber acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms incoming ROS data into a
  // trajectory and then passes it to the TrajectoryWarden to manage.
  class TrajectorySubscriberNode {
    private:
      // Trajectory warden manages multi-threaded access to trajectory data
      std::shared_ptr<TrajectoryWarden> warden_;

      // ROS node handle
      ros::NodeHandle node_handle_;

      // ROS subscriebr
      ros::Subscriber subscriber_;

      // Key to be passed on to the trajectory warden
      std::string key_;

      // Subscriber callback. Extracts ROS data and converts it into a
      // Trajectory instance to be passed on to the trajectory warden
      void SubscriberCallback(const mg_msgs::PVAYStampedTrajectory& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      TrajectorySubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<TrajectoryWarden> warden);
  };
}
