// Author: Tucker Haydon

#pragma once

#include <memory>
#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>

namespace path_planning {
  class PolygonPublisherNode {
    private:
      std::shared_ptr<ros::NodeHandle> nh_;
      ros::Publisher polygon_pub_;

    public:
      PolygonPublisherNode(int argc = 0, char** argv = nullptr);

      // Intentional pass-by-value. Funtion could be called by another thread
      // and need to ensure data does not expire on this thread
      bool Publish(const jsk_recognition_msgs::PolygonArray polygons);
  };

}
