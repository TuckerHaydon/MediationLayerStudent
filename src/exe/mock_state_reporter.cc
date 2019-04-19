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
#include <random>
#include <unordered_map>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "yaml-cpp/yaml.h"
#include "map3d.h"

#include "quad_state.h"
#include "quad_state_publisher_node.h"
#include "quad_state_guard.h"
#include "quad_state_dispatcher.h"

using namespace mediation_layer;

namespace { 
  // Signal variable and handler
  volatile std::sig_atomic_t kill_program;
  void SigIntHandler(int sig) {
    kill_program = 1;
  }

  QuadState Propagate(const QuadState& quad_state, double dt) {
    const Eigen::Vector3d old_pos = quad_state.Position();
    const Eigen::Vector3d old_vel = quad_state.Velocity();
    const Eigen::Vector4d old_orientation = quad_state.Orientation();
    const Eigen::Vector3d old_twist = quad_state.Twist();

    const Eigen::Vector3d new_pos = old_pos + old_vel * dt;
    const Eigen::Vector3d new_vel = old_vel;
    const Eigen::Vector4d new_orientation = old_orientation;
    const Eigen::Vector3d new_twist = old_twist;

    return QuadState((
          Eigen::Vector<double, 13>() 
          << new_pos, new_vel, new_orientation, new_twist
          ).finished());
  }
}


int main(int argc, char** argv) {
  // Configure sigint handler
  std::signal(SIGINT, SigIntHandler);

  // Start ROS
  ros::init(argc, argv, "mock_state_reporter", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/mediation_layer/");

  // Load map
  std::string map_file_path;
  if(false == nh.getParam("map_file_path", map_file_path)) {
    std::cerr << "Required parameter not found on server: map_file_path" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const YAML::Node node = YAML::LoadFile(map_file_path);
  const Map3D map = node["map"].as<Map3D>();

  // Load quad state topics
  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::unordered_map<std::string, std::shared_ptr<QuadStatePublisherNode>> quad_state_publishers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    quad_state_publishers[quad_name]
        = std::make_shared<QuadStatePublisherNode>(topic);
  }

  std::vector<std::pair<std::string, QuadState>> quad_states;
  for(const auto& kv: quad_state_topics) {
     std::random_device rd;
     std::mt19937 gen(rd());
     std::uniform_real_distribution<> dist(-1.0, 1.0);
    QuadState quad_state(Eigen::Vector<double, 13>(
          0,0,1,
          dist(gen),dist(gen),0,
          1,0,0,0,
          0,0,0
          ));
    quad_states.push_back(std::make_pair(kv.first, quad_state));
  }
  
  // Publish state
  std::thread quad_state_publisher_thread(
      [&]() {
        while(true) {
          if(true == kill_program) {
            break;
          } else {
            const size_t dt_ms = 2;
            const double dt = dt_ms / 1000.0;

            for(auto& p: quad_states) {
              const std::string& topic = p.first;

              // Propagate the quad state in time
              p.second = Propagate(p.second, dt);

              // Publish the quad state
              quad_state_publishers[p.first]->Publish(p.second);
            }


            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
          }
        }
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
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();
  quad_state_publisher_thread.join();


  return EXIT_SUCCESS;
}

