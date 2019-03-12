// Author: Tucker Haydon

#include "polygon_publisher_node.h"

namespace mediation_layer {
  PolygonPublisherNode::PolygonPublisherNode(int argc, char** argv) {
    // ros::init(argc, argv, "PolygonPublisherNode", 
    //     ros::init_options::NoSigintHandler);
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    this->polygon_pub_ = this->nh_->advertise<jsk_recognition_msgs::PolygonArray>("/polygon", 1);
  }

  bool PolygonPublisherNode::Publish(const jsk_recognition_msgs::PolygonArray polygons) {
    this->polygon_pub_.publish(polygons);
    std::cout << "Published" << std::endl;
    return true;
  }
}
