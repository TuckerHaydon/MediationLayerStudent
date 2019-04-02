// Author: Tucker Haydon

#pragma once

#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "publisher_guard.h"

namespace mediation_layer {
  class MarkerPublisherNode {
    private:
      std::shared_ptr<PublisherGuard<visualization_msgs::Marker>> publisher_guard_;

    public:
      MarkerPublisherNode(const std::string& topic);

      void Publish(const visualization_msgs::Marker& msg);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline MarkerPublisherNode::MarkerPublisherNode(const std::string& topic) {
    this->publisher_guard_ 
      = std::make_shared<PublisherGuard<visualization_msgs::Marker>>(topic);
  }

  inline void MarkerPublisherNode::Publish(const visualization_msgs::Marker& msg) {
    this->publisher_guard_->Publish(msg);
  }
}
