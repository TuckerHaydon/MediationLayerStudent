// Author: Tucker Haydon

#include <cstdlib>
#include <string>
#include <vector>
#include <memory>
#include <csignal>
#include <Eigen/Dense>
#include <ros/ros.h>

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
#include "ap_test.h"

using namespace mediation_layer;

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

  // Initialize the QuadStateWarden
  auto quad_state_warden  = std::make_shared<QuadStateWarden3D>();
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    quad_state_warden->Register(quad_name);
  }

  // Pipe ROS data into the QuadStateWarden
  std::vector<std::shared_ptr<QuadStateSubscriberNode3D>> quad_state_subscribers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    quad_state_subscribers.push_back(
        std::make_shared<QuadStateSubscriberNode3D>(
            topic, 
            quad_name, 
            quad_state_warden));
  }

  // Initialize the GameSnapshot
  auto game_snapshot = std::make_shared<GameSnapshot3D>(
      blue_quad_names,
      red_quad_names,
      quad_state_warden,
      GameSnapshot3D::Options());

  // Initialize the TrajectoryPublishers
  std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode3D>> proposed_trajectory_publishers;
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    proposed_trajectory_publishers[quad_name] = 
      std::make_shared<TrajectoryPublisherNode3D>(topic);
  }

  // Initialize the TrajectoryWarden
  auto trajectory_warden_out = std::make_shared<TrajectoryWarden3D>();

  // Pipe the TrajectoryWarden to the TrajectoryPublishers
  auto trajectory_dispatcher = std::make_shared<TrajectoryDispatcher3D>();
  std::thread trajectory_dispatcher_thread([&](){
      trajectory_dispatcher->Run(trajectory_warden_out, proposed_trajectory_publishers);
      });

  // The AutonomyProtocol
  std::shared_ptr<AutonomyProtocol3D> autonomy_protocol = std::make_shared<TestAP>(
      blue_quad_names, 
      red_quad_names,
      game_snapshot,
      trajectory_warden_out);

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
        autonomy_protocol->Stop();
        trajectory_dispatcher->Stop();
        quad_state_warden->Stop();
        trajectory_warden_out->Stop();
      });

  // Wait for program termination via ctl-c
  kill_thread.join();
  ap_thread.join();
  trajectory_dispatcher_thread.join();

  return EXIT_SUCCESS;
}
