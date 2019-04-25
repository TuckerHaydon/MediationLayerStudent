// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state.h"
#include "publisher_guard.h"
#include "nav_msgs/Odometry.h"

namespace game_engine {
  // QuadStatePublisherNode acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms internal QuadState into a
  // ROS quad_state and publishes it.
  //
  // Wraps the publisher in a publisher guard to ensure that instances of this
  // class may be safely used across multiple threads
  class QuadStatePublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<nav_msgs::Odometry>> publisher_guard_;
      
    public:
      // Constructor.
      QuadStatePublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const QuadState& quad_state);
  };
}
