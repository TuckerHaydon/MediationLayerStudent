// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory_warden.h"
#include "trajectory.h"
#include "mg_msgs/PVAYStampedTrajectory.h"

namespace mediation_layer {
  // Trajectory subscriber acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms incoming ROS data into a
  // trajectory and then passes it to the TrajectoryWarden to manage.
  template <size_t T>
  class TrajectorySubscriberNode {
    private:
      std::shared_ptr<TrajectoryWarden<T>> warden_;
      ros::NodeHandle node_handle_;
      ros::Subscriber subscriber_;
      std::string key_;

      void SubscriberCallback(const mg_msgs::PVAYStampedTrajectory& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      TrajectorySubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<TrajectoryWarden<T>> warden);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  TrajectorySubscriberNode<T>::TrajectorySubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<TrajectoryWarden<T>> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = ros::NodeHandle("~");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &TrajectorySubscriberNode<T>::SubscriberCallback, 
        this);
  }

  template <size_t T>
  void TrajectorySubscriberNode<T>::SubscriberCallback(const mg_msgs::PVAYStampedTrajectory& msg) {
      // Required data structure. Formatted as follows:
      //   [ pos(T), vel(T), acc(T), yaw(1), time(1)]
      std::vector<
        Eigen::Vector<double, 3*T + 2>, 
        Eigen::aligned_allocator<Eigen::Vector<double, 3*T + 2>>> data;
    for(const mg_msgs::PVAYStamped& instant: msg.trajectory) {
      Eigen::Vector<double, 3*T + 2> local_instant;
      if(T == 2) {
        local_instant(0) = instant.pos.x;
        local_instant(1) = instant.pos.y;
        local_instant(2) = instant.vel.linear.x;
        local_instant(3) = instant.vel.linear.y;
        local_instant(4) = instant.acc.linear.x;
        local_instant(5) = instant.acc.linear.y;
        local_instant(6) = instant.yaw;
        local_instant(7) = instant.header.stamp.sec + instant.header.stamp.nsec / 1e9;
      } else
      if (T == 3) {
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
        local_instant(10) = instant.header.stamp.sec + instant.header.stamp.nsec / 1e9;
      }
      data.push_back(local_instant);
    }

    this->warden_->Write(this->key_, Trajectory<T>(data));
  }

  using TrajectorySubscriberNode2D = TrajectorySubscriberNode<2>;
  using TrajectorySubscriberNode3D = TrajectorySubscriberNode<3>;
}
