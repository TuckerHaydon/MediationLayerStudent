// Author: Tucker Haydon

#include "marker_publisher_node.h"

namespace mediation_layer {
  MarkerPublisherNode::MarkerPublisherNode(const int queue_size=100) {
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    *(this->marker_pub_) = this->nh_->advertise<visualization_msgs::Marker>("/markers", queue_size);
  }

  bool MarkerPublisherNode::Publish(const visualization_msgs::Marker marker) {
    this->marker_pub_->publish(marker);
    return true;
  }
}
