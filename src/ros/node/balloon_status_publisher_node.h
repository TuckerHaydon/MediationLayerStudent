// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state.h"
#include "publisher_guard.h"
#include "balloon_status.h"

#include "mg_msgs/BalloonStatus.h"

namespace game_engine {
  class BalloonStatusPublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<mg_msgs::BalloonStatus>> publisher_guard_;
      
    public:
      // Constructor.
      BalloonStatusPublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const BalloonStatus& balloon_status);
  };
}
