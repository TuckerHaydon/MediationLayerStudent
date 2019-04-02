// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory.h"
#include "publisher_guard.h"
#include "mg_msgs/PVAYStampedTrajectory.h"

namespace mediation_layer {
  // Trajectory publisher acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms internal trajectory into a
  // ROS trajectory and publishes it.
  template <size_t T>
  class TrajectoryPublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<mg_msgs::PVAYStampedTrajectory>> publisher_guard_;
      
    public:
      // Constructor.
      TrajectoryPublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const Trajectory<T>& trajectory);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline TrajectoryPublisherNode<T>::TrajectoryPublisherNode(
      const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<mg_msgs::PVAYStampedTrajectory>>(topic);
  }

  template <size_t T>
  inline void TrajectoryPublisherNode<T>::Publish(const Trajectory<T>& trajectory) {
    mg_msgs::PVAYStampedTrajectory msg;
    for(size_t idx = 0; idx < trajectory.Size(); ++idx) {
      mg_msgs::PVAYStamped instant;
      if(T == 2) {
        instant.pos.x = trajectory.Position(idx).x();
        instant.pos.y = trajectory.Position(idx).y();
        instant.vel.linear.x = trajectory.Velocity(idx).x();
        instant.vel.linear.y = trajectory.Velocity(idx).y();
        instant.acc.linear.x = trajectory.Acceleration(idx).x();
        instant.acc.linear.y = trajectory.Acceleration(idx).y();
        instant.yaw = trajectory.Yaw(idx);
        instant.header.stamp.sec = std::floor(trajectory.Time(idx));
        instant.header.stamp.nsec 
          = (trajectory.Time(idx) - std::floor(trajectory.Time(idx))) * 1e9;
      } else
      if(T == 3) {
        instant.pos.x = trajectory.Position(idx).x();
        instant.pos.y = trajectory.Position(idx).y();
        instant.pos.z = trajectory.Position(idx).z();
        instant.vel.linear.x = trajectory.Velocity(idx).x();
        instant.vel.linear.y = trajectory.Velocity(idx).y();
        instant.vel.linear.z = trajectory.Velocity(idx).z();
        instant.acc.linear.x = trajectory.Acceleration(idx).x();
        instant.acc.linear.y = trajectory.Acceleration(idx).y();
        instant.acc.linear.z = trajectory.Acceleration(idx).z();
        instant.yaw = trajectory.Yaw(idx);
        instant.header.stamp.sec = std::floor(trajectory.Time(idx));
        instant.header.stamp.nsec 
          = (trajectory.Time(idx) - std::floor(trajectory.Time(idx))) * 1e9;
      }

      msg.trajectory.push_back(instant);
    }
    this->publisher_guard_->Publish(msg);
  }

  using TrajectoryPublisherNode2D = TrajectoryPublisherNode<2>;
  using TrajectoryPublisherNode3D = TrajectoryPublisherNode<3>;
}
