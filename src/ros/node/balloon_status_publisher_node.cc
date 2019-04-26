// Author: Tucker Haydon

#include "balloon_status_publisher_node.h"
#include <chrono>

namespace game_engine {
  BalloonStatusPublisherNode::BalloonStatusPublisherNode(
      const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<mg_msgs::BalloonStatus>>(topic);
  }

  void BalloonStatusPublisherNode::Publish(const BalloonStatus& balloon_status) {
    // const auto t_seconds 
    //   = std::chrono::duration_cast<std::chrono::seconds>(balloon_status.pop_time.time_since_epoch());
    // const auto t_nanoseconds 
    //   = std::chrono::duration_cast<std::chrono::nanoseconds>(balloon_status.pop_time.time_since_epoch())
    //   - t_seconds;

    mg_msgs::BalloonStatus msg;
    msg.header.frame_id = "world";
    // msg.header.stamp.sec = t_seconds.count();
    // msg.header.stamp.nsec = t_nanoseconds.count();
    msg.header.stamp.sec = 0;
    msg.header.stamp.nsec = 0;
    msg.popped.data = balloon_status.popped;
    msg.popper.data = balloon_status.popper;

    this->publisher_guard_->Publish(msg);
  }
}
