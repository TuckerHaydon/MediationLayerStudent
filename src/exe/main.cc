// Author: Tucker Haydon

#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <memory>
#include <thread>
#include <csignal>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "yaml-cpp/yaml.h"
#include "map2d.h"

#include "trajectory_warden.h"
#include "trajectory.h"
#include "trajectory_subscriber_node.h"
#include "trajectory_publisher_node.h"

#include "state_warden.h"
#include "quad_state.h"
#include "state_subscriber_node.h"

#include "trajectory_dispatcher.h"
#include "mediation_layer.h"

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
  ros::init(argc, argv, "mediation_layer", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  // YAML config
  std::string map_file_path;
  if(false == nh.getParam("map_file_path", map_file_path)) {
    std::cerr << "Required parameter not found on server: map_file_path" << std::endl;
    std::exit(1);
  }

  const YAML::Node node = YAML::LoadFile(map_file_path);
  const Map2D map = node["map"].as<Map2D>();

  // Mediation layer state
  auto proposed_trajectory_warden = std::make_shared<TrajectoryWarden<2>>();
  auto updated_trajectory_warden = std::make_shared<TrajectoryWarden<2>>();

  // Initialize subscribers
  // Load the subscriber topics
  std::map<std::string, std::string> proposed_trajectory_topics;
  if(false == nh.getParam("proposed_trajectory_topics", proposed_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: proposed_trajectory_topics" << std::endl;
    std::exit(1);
  }

  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(1);
  }

  // Initialize the TrajectoryWardens. The TrajectoryWarden enables safe,
  // multi-threaded access to trajectory data. Internal components that require
  // access to proposed and updated trajectories should request access through
  // TrajectoryWarden.
  auto trajectory_warden_in  = std::make_shared<TrajectoryWarden2D>();
  auto trajectory_warden_out = std::make_shared<TrajectoryWarden2D>();
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    trajectory_warden_in->Register(quad_name);
    trajectory_warden_out->Register(quad_name);
  }

  // For every quad, subscribe to its corresponding proposed_trajectory topic
  std::unordered_map<std::string, std::shared_ptr<TrajectorySubscriberNode2D>> trajectory_subscribers;
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    trajectory_subscribers[quad_name] = 
        std::move(std::make_shared<TrajectorySubscriberNode2D>(
            topic, 
            quad_name, 
            trajectory_warden_in));
  }

  // Initialize publishers
  // Load the publisher topics
  std::map<std::string, std::string> updated_trajectory_topics;
  if(false == nh.getParam("updated_trajectory_topics", updated_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: updated_trajectory_topics" << std::endl;
    std::exit(1);
  }

  // For every quad, publish to its corresponding updated_trajectory topic
  std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode2D>> trajectory_publishers;
  for(const auto& kv: updated_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    trajectory_publishers[quad_name] = 
      std::move(std::make_shared<TrajectoryPublisherNode2D>(
            topic));
  }

  // Initialize the StateWarden. The StateWarden enables safe, multi-threaded
  // access to quadcopter state data. Internal components that require access to
  // state data should request access through StateWarden.
  auto state_warden  = std::make_shared<StateWarden2D>();
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    state_warden->Register(quad_name);
  }

  // For every quad, subscribe to its corresponding state topic
  std::vector<std::shared_ptr<StateSubscriberNode2D>> state_subscribers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    state_subscribers.push_back(
        std::move(std::make_shared<StateSubscriberNode2D>(
            topic, 
            quad_name, 
            state_warden)));
  }

  // TrajectoryDispatcher thread. TrajectoryDispatcher pipes data from
  // TrajectoryWarden to its corresponding trajectory publisher. Runs as a
  // separate thread and maintains a thread pool. Each thread in the pool is
  // assigned a particular trajectory and blocks until that trajectory is modified.
  // Once modified, it moves the data into the corresponding publisher queue.
  auto trajectory_dispatcher = std::make_shared<TrajectoryDispatcher2D>();
  std::thread trajectory_dispatcher_thread([&](){
      trajectory_dispatcher->Run(trajectory_warden_out, trajectory_publishers);
      });

  // Mediation layer thread. The mediation layer runs continuously, forward
  // integrating the proposed trajectories and modifying them so that the
  // various agents will not crash into each other. Data is asynchonously read
  // and written from the TrajectoryWardens
  auto mediation_layer = std::make_shared<MediationLayer2D>();
  std::thread mediation_layer_thread(
      [&]() {
        mediation_layer->Run(
            trajectory_warden_in, 
            trajectory_warden_out, 
            state_warden);
      });

  // Kill program thread. This thread sleeps for a second and then checks if the
  // 'kill_program' variable has been set. If it has, it shuts ros down and
  // sends stop signals to any other threads that might be running.
  std::thread kill_thread(
      [&]() {
        while(true) {
          if(true == kill_program) {
            break;
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          }
        }
        mediation_layer->Stop();
        trajectory_dispatcher->Stop();
        ros::shutdown();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();

  // Wait for other threads to die
  trajectory_dispatcher_thread.join();
  mediation_layer_thread.join();

  return EXIT_SUCCESS;
}

