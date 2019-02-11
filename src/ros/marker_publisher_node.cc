// Author: Tucker Haydon

#include <visualization_msgs/Marker.h>

#include "marker_publisher_node.h"

namespace path_planning {
  MarkerPublisherNode::MarkerPublisherNode(int argc, char** argv) {
    ros::init(argc, argv, "MarkerPublisherNode", 
        ros::init_options::NoSigintHandler);
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    this->marker_pub_ = this->nh_->advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  }

  bool MarkerPublisherNode::Publish(const visualization_msgs::Marker marker) {
    this->marker_pub_.publish(marker);
    return true;
  }
}
