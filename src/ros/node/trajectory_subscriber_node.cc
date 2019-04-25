// Author: Tucker Haydon

#include "trajectory_subscriber_node.h"

namespace game_engine {
  TrajectorySubscriberNode::TrajectorySubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<TrajectoryWarden> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = ros::NodeHandle("/mediation_layer/");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &TrajectorySubscriberNode::SubscriberCallback, 
        this);
  }

  void TrajectorySubscriberNode::SubscriberCallback(const mg_msgs::PVAYStampedTrajectory& msg) {
      // Required data structure. Formatted as follows:
      //   [ pos(3), vel(3), acc(3), yaw(1), time(1)]
      std::vector<
        Eigen::Matrix<double, 11, 1>, 
        Eigen::aligned_allocator<Eigen::Matrix<double, 11, 1>>> data;
    for(const mg_msgs::PVAYStamped& instant: msg.trajectory) {
      Eigen::Matrix<double, 11, 1> local_instant;
      local_instant(0) = instant.pos.x;
      local_instant(1) = instant.pos.y;
      local_instant(2) = instant.pos.z;
      local_instant(3) = instant.vel.linear.x;
      local_instant(4) = instant.vel.linear.y;
      local_instant(5) = instant.vel.linear.z;
      local_instant(6) = instant.acc.linear.x;
      local_instant(7) = instant.acc.linear.y;
      local_instant(8) = instant.acc.linear.z;
      local_instant(9) = instant.yaw;
      local_instant(10) = instant.header.stamp.sec + (double)instant.header.stamp.nsec / 1e9;
      data.push_back(local_instant);
    }

    this->warden_->Write(this->key_, Trajectory(data));
  }
}

