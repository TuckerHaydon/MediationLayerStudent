// Author: Tucker Haydon

#include "balloon_status_subscriber_node.h"

namespace game_engine {
  BalloonStatusSubscriberNode::BalloonStatusSubscriberNode(
      const std::string& topic, 
      std::shared_ptr<BalloonStatus> balloon_status) {
    this->balloon_status_ = balloon_status;
    this->node_handle_ = ros::NodeHandle("/mediation_layer/");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &BalloonStatusSubscriberNode::SubscriberCallback, 
        this);
  }

  void BalloonStatusSubscriberNode::SubscriberCallback(const mg_msgs::BalloonStatus& msg) {
    BalloonStatus balloon_status {
      .popped = msg.popped.data,
      .popper = msg.popper.data,
      .pop_time = std::chrono::system_clock::now() 
    };
    *(this->balloon_status_) = balloon_status;
  }
}
