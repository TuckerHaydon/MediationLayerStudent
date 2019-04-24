// Author: Tucker Haydon

#include "trajectory_publisher_node.h"

namespace mediation_layer {
  TrajectoryPublisherNode::TrajectoryPublisherNode(
      const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<mg_msgs::PVAYStampedTrajectory>>(topic);
  }

  void TrajectoryPublisherNode::Publish(const Trajectory& trajectory) {
    mg_msgs::PVAYStampedTrajectory msg;
    for(size_t idx = 0; idx < trajectory.Size(); ++idx) {
      mg_msgs::PVAYStamped instant;
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
      instant.header.frame_id = "world";
      instant.header.stamp.sec = std::floor(trajectory.Time(idx));
      instant.header.stamp.nsec 
        = (trajectory.Time(idx) - std::floor(trajectory.Time(idx))) * 1e9;

      msg.trajectory.push_back(instant);
    }
    this->publisher_guard_->Publish(msg);
  }
}

