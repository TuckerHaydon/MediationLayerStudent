// Author: Tucker Haydon

#pragma once

#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "publisher_guard.h"

namespace mediation_layer {
  // ROS node that publishers marker messages. Wraps the publisher in a
  // publisher guard to ensure that instances of this class may be used in
  // across multiple threads
  class MarkerPublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<visualization_msgs::Marker>> publisher_guard_;

    public:
      // Constructor
      MarkerPublisherNode(const std::string& topic);

      // Publish the message
      void Publish(const visualization_msgs::Marker& msg);
  };
}
