// Author: Tucker Haydon

#pragma once

#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace mediation_layer {
  class MarkerPublisherNode {
    private:
      std::shared_ptr<ros::NodeHandle> nh_;
      std::shared_ptr<ros::Publisher> marker_pub_;

    public:
      MarkerPublisherNode(const int queue_size=100);

      // Intentional pass-by-value. Funtion could be called by another thread
      // and need to ensure data does not expire on this thread
      bool Publish(const visualization_msgs::Marker marker);
  };

}
