// Author: Tucker Haydon

#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <memory>
#include <thread>
#include <csignal>
#include <utility>
#include <sstream>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "yaml-cpp/yaml.h"
#include "map3d.h"

#include "trajectory_warden.h"
#include "trajectory.h"
#include "trajectory_subscriber_node.h"

#include "quad_state_warden.h"
#include "quad_state.h"
#include "quad_state_publisher_node.h"

#include "trajectory_dispatcher.h"
#include "mediation_layer.h"
#include "physics_simulator.h"

#include "quad_state_guard.h"

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
  ros::init(argc, argv, "physics_simulator", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/mediation_layer/");

  std::map<std::string, std::string> updated_trajectory_topics;
  if(false == nh.getParam("updated_trajectory_topics", updated_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: updated_trajectory_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  auto trajectory_warden_in  = std::make_shared<TrajectoryWarden>();
  for(const auto& kv: updated_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    trajectory_warden_in->Register(quad_name);
  }

  std::unordered_map<std::string, std::shared_ptr<TrajectorySubscriberNode>> trajectory_subscribers;
  for(const auto& kv: updated_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    trajectory_subscribers[quad_name] = 
        std::make_shared<TrajectorySubscriberNode>(
            topic, 
            quad_name, 
            trajectory_warden_in);
  }

  std::unordered_map<std::string, std::shared_ptr<QuadStatePublisherNode>> quad_state_publishers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    quad_state_publishers[quad_name]
        = std::make_shared<QuadStatePublisherNode>(topic);
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

  PhysicsSimulator::Options physics_simulator_options;
  physics_simulator_options.initial_quad_positions = initial_quad_positions;
  auto physics_simulator = std::make_shared<PhysicsSimulator>(physics_simulator_options);
  std::thread physics_simulator_thread(
      [&]() {
        physics_simulator->Run(trajectory_warden_in, quad_state_publishers);
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
        ros::shutdown();

        trajectory_warden_in->Stop();
        physics_simulator->Stop();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();
  physics_simulator_thread.join();

  return EXIT_SUCCESS;
}
