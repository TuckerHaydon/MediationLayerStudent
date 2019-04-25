// Author: Tucker Haydon

#include "marker_publisher_node.h"

namespace game_engine {
  MarkerPublisherNode::MarkerPublisherNode(const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<visualization_msgs::Marker>>(topic);
  }

  void MarkerPublisherNode::Publish(const visualization_msgs::Marker& msg) {
    this->publisher_guard_->Publish(msg);
  }
}
