// Author: Tucker Haydon

#include <cstdlib>
#include <string>
#include <vector>
#include <memory>
#include <csignal>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sstream>

#include "yaml-cpp/yaml.h"
#include "map3d.h"

#include "trajectory_warden.h"
#include "trajectory.h"
#include "trajectory_publisher_node.h"

#include "quad_state_warden.h"
#include "quad_state.h"
#include "quad_state_subscriber_node.h"

#include "trajectory_dispatcher.h"

#include "quad_state_guard.h"

#include "game_snapshot.h"
#include "example_autonomy_protocol.h"
#include "map3d.h"

using namespace game_engine;

namespace { 
  // Signal variable and handler
  volatile std::sig_atomic_t kill_program;
  void SigIntHandler(int sig) {
    kill_program = 1;
  }
}


int main(int argc, char** argv) {
  // Configure sigint handler
  std::signal(SIGINT, SigIntHandler);

  // Start ROS
  ros::init(argc, argv, "autonomy_protocol", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/mediation_layer/");

  // Read ROS data
  std::string map_file_path;
  if(false == nh.getParam("map_file_path", map_file_path)) {
    std::cerr << "Required parameter not found on server: map_file_path" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const YAML::Node node = YAML::LoadFile(map_file_path);
  const Map3D map3d = node["map"].as<Map3D>();

  std::map<std::string, std::string> team_assignments;
  if(false == nh.getParam("team_assignments", team_assignments)) {
    std::cerr << "Required parameter not found on server: team_assignments" << std::endl;
    std::exit(1);
  }

  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::map<std::string, std::string> proposed_trajectory_topics;
  if(false == nh.getParam("proposed_trajectory_topics", proposed_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: proposed_trajectory_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::vector<double> blue_balloon_position_vector;
  if(false == nh.getParam("blue_balloon_position", blue_balloon_position_vector)) {
    std::cerr << "Required parameter not found on server: blue_balloon_position" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  const Eigen::Vector3d blue_balloon_position(
      blue_balloon_position_vector[0],
      blue_balloon_position_vector[1],
      blue_balloon_position_vector[2]);

  std::vector<double> red_balloon_position_vector;
  if(false == nh.getParam("red_balloon_position", red_balloon_position_vector)) {
    std::cerr << "Required parameter not found on server: red_balloon_position" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  const Eigen::Vector3d red_balloon_position(
      red_balloon_position_vector[0],
      red_balloon_position_vector[1],
      red_balloon_position_vector[2]);

  std::map<
    std::string, 
    Eigen::Vector3d, 
    std::less<std::string>, 
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> balloon_map;

  balloon_map["red"] = red_balloon_position;
  balloon_map["blue"] = red_balloon_position;

  // Team Assignments
  std::vector<std::string> red_quad_names;
  std::vector<std::string> blue_quad_names;
  for(const auto& kv: team_assignments) {
    if(kv.second == "red") {
      red_quad_names.push_back(kv.first);
    }
    if(kv.second == "blue") {
      blue_quad_names.push_back(kv.first);
    }
  }

  // Parse the initial quad positions
  std::map<std::string, std::string> initial_quad_positions_string;
  if(false == nh.getParam("initial_quad_positions", initial_quad_positions_string)) {
    std::cerr << "Required parameter not found on server: initial_quad_positions" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::map<
    std::string, 
    Eigen::Vector3d, 
    std::less<std::string>, 
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> initial_quad_positions;
  for(const auto& kv: initial_quad_positions_string) {
    const std::string& quad_name = kv.first;
    const std::string& quad_position_string = kv.second;
    std::stringstream ss(quad_position_string);
    double x,y,z;
    ss >> x >> y >> z;
    initial_quad_positions[quad_name] = Eigen::Vector3d(x,y,z);
  }

  // Initialize the QuadStateWarden
  auto quad_state_warden  = std::make_shared<QuadStateWarden>();
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    quad_state_warden->Register(quad_name);

    const Eigen::Vector3d initial_quad_pos = initial_quad_positions[quad_name];
    const QuadState initial_quad_state(Eigen::Matrix<double, 13, 1>(
          initial_quad_pos(0), initial_quad_pos(1), initial_quad_pos(2),
          0,0,0,
          1,0,0,0,
          0,0,0));
    quad_state_warden->Write(quad_name,initial_quad_state);
  }

  // Pipe ROS data into the QuadStateWarden
  std::vector<std::shared_ptr<QuadStateSubscriberNode>> quad_state_subscribers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    quad_state_subscribers.push_back(
        std::make_shared<QuadStateSubscriberNode>(
            topic, 
            quad_name, 
            quad_state_warden));
  }

  // Initialize the GameSnapshot
  auto game_snapshot = std::make_shared<GameSnapshot>(
      blue_quad_names,
      red_quad_names,
      quad_state_warden,
      GameSnapshot::Options());

  // Initialize the TrajectoryPublishers
  std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode>> proposed_trajectory_publishers;
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    proposed_trajectory_publishers[quad_name] = 
      std::make_shared<TrajectoryPublisherNode>(topic);
  }

  // Initialize the TrajectoryWarden
  auto trajectory_warden_out = std::make_shared<TrajectoryWarden>();
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    trajectory_warden_out->Register(quad_name);
  }

  // Pipe the TrajectoryWarden to the TrajectoryPublishers
  auto trajectory_dispatcher = std::make_shared<TrajectoryDispatcher>();
  std::thread trajectory_dispatcher_thread([&](){
      trajectory_dispatcher->Run(trajectory_warden_out, proposed_trajectory_publishers);
      });

  // The AutonomyProtocol
  std::shared_ptr<AutonomyProtocol> autonomy_protocol = std::make_shared<ExampleAutonomyProtocol>(
      blue_quad_names, 
      red_quad_names,
      game_snapshot,
      trajectory_warden_out,
      map3d,
      balloon_map);

  // Start the autonomy protocol
  std::thread ap_thread(
      [&]() {
        autonomy_protocol->Run();
      });
  
  // Start the kill thread
  std::thread kill_thread(
      [&]() {
        while(true) {
          if(true == kill_program) {
            break;
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          }
        }

        ros::shutdown();

        autonomy_protocol->Stop();
        trajectory_dispatcher->Stop();
        quad_state_warden->Stop();
        trajectory_warden_out->Stop();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();
  ap_thread.join();
  trajectory_dispatcher_thread.join();

  return EXIT_SUCCESS;
}
