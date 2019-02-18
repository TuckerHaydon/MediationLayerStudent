// Author: Tucker Haydon

#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>

#include "yaml-cpp/yaml.h"
#include "map2d.h"
#include "mediation_layer2d.h"

using namespace mediation_layer;

void DummyFunction(const std_msgs::String::ConstPtr& msg) {

}

int main(int argc, char** argv) {
  /*
   * Start ROS
   */
  ros::init(argc, argv, "mediation_layer");
  ros::NodeHandle nh("~");


  /*
   * Configure
   */
  std::string map_file_path;
  if(false == nh.getParam("map_file_path", map_file_path)) {
    std::cerr << "Required parameter not found on server: map_file_path" << std::endl;
    std::exit(1);
  }

  const YAML::Node node = YAML::LoadFile(map_file_path);
  const Map2D map = node["map"].as<Map2D>();


  /*
   * Start mediation layer
   */
  State2D proposed_state, updated_state;
  MediationLayer2D mediation_layer(map);

  std::thread mediation_layer_thread(
      // Capture references
      [&]() {
        mediation_layer.Run(proposed_state, updated_state);
      });


  /*
   * Start the subscribers
   */
  // Load the subscriber topics
  std::map<std::string, std::string> proposed_trajectory_topics;
  if(false == nh.getParam("proposed_trajectory_topics", proposed_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: proposed_trajectory_topics" << std::endl;
    std::exit(1);
  }

  // For every quad, subscribe to its corresponding proposed_trajectory topic
  std::vector<ros::Subscriber> subscribers;
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    subscribers.push_back(nh.subscribe(topic, 1, DummyFunction));
  }
  

  /*
   * Spin and process subscription messages
   */
  ros::spin();


  /*
   * Wait for thread termination
   */
  mediation_layer_thread.join();

  return EXIT_SUCCESS;
}
