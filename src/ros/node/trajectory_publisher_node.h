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
  //
  // Wraps the publisher in a publisher guard to ensure that instances of this
  // class may be safely used across multiple threads
  class TrajectoryPublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<mg_msgs::PVAYStampedTrajectory>> publisher_guard_;
      
    public:
      // Constructor.
      TrajectoryPublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const Trajectory& trajectory);
  };
}
