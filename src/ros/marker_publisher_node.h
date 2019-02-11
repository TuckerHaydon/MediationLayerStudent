// Author: Tucker Haydon

#pragma once

#include <memory>
#include <ros/ros.h>

namespace path_planning {
  class MarkerPublisherNode {
    private:
      std::shared_ptr<ros::NodeHandle> nh_;
      ros::Publisher marker_pub_;

    public:
      MarkerPublisherNode(int argc, char** argv);

      // Intentional pass-by-value. Funtion could be called by another thread
      // and need to ensure data does not expire on this thread
      bool Publish(const visualization_msgs::Marker marker);
  };

}
