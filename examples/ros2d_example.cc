// Author: Tucker Haydon

#include <cstdlib>
#include <thread>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

// #include "marker_publisher_node.h"

// using namespace path_planning; 

int main(int argc, char** argv) {
  // MarkerPublisherNode marker_pub(argc, argv);
  // uint32_t shape = visualization_msgs::Marker::CUBE;

  // while(true) {
  //   visualization_msgs::Marker marker;
  //     
  //   marker.header.frame_id = "/inertial";
  //   marker.header.stamp = ros::Time::now();
  //   
  //   marker.ns = "basic_shapes";
  //   marker.id = 0;
  //   marker.type = shape; 
  //   marker.action = visualization_msgs::Marker::ADD;
  //   
  //   marker.pose.position.x = 0;
  //   marker.pose.position.y = 0;
  //   marker.pose.position.z = 0;
  //   marker.pose.orientation.x = 0.0;
  //   marker.pose.orientation.y = 0.0;
  //   marker.pose.orientation.z = 0.0;
  //   marker.pose.orientation.w = 1.0;
  //   
  //   marker.scale.x = 1.0;
  //   marker.scale.y = 1.0;
  //   marker.scale.z = 1.0;
  //   
  //   marker.color.r = 0.0f;
  //   marker.color.g = 1.0f;
  //   marker.color.b = 0.0f;
  //   marker.color.a = 1.0;

  //   marker.lifetime = ros::Duration();
  //   
  //   marker_pub.Publish(marker);
 
  //   switch (shape)
  //   {
  //   case visualization_msgs::Marker::CUBE:
  //     shape = visualization_msgs::Marker::SPHERE;
  //     break;
  //   case visualization_msgs::Marker::SPHERE:
  //     shape = visualization_msgs::Marker::ARROW;
  //     break;
  //   case visualization_msgs::Marker::ARROW:
  //     shape = visualization_msgs::Marker::CYLINDER;
  //     break;
  //   case visualization_msgs::Marker::CYLINDER:
  //     shape = visualization_msgs::Marker::CUBE;
  //     break;
  //   }
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // }

  return EXIT_SUCCESS;
}
