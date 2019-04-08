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

#include <Eigen/Dense>
#include <ros/ros.h>

#include "yaml-cpp/yaml.h"
#include "map3d.h"

#include "trajectory_warden.h"
#include "trajectory.h"
#include "trajectory_subscriber_node.h"
#include "trajectory_publisher_node.h"

#include "quad_state_warden.h"
#include "quad_state.h"
#include "quad_state_subscriber_node.h"

#include "trajectory_dispatcher.h"
#include "mediation_layer.h"

#include "quad_state_guard.h"

#include "view_manager.h"

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
    std::exit(EXIT_FAILURE);
  }

  const YAML::Node node = YAML::LoadFile(map_file_path);
  const Map3D map = node["map"].as<Map3D>();

  // Initialize subscribers
  // Load the subscriber topics
  std::map<std::string, std::string> proposed_trajectory_topics;
  if(false == nh.getParam("proposed_trajectory_topics", proposed_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: proposed_trajectory_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Initialize the TrajectoryWardens. The TrajectoryWarden enables safe,
  // multi-threaded access to trajectory data. Internal components that require
  // access to proposed and updated trajectories should request access through
  // TrajectoryWarden.
  auto trajectory_warden_in  = std::make_shared<TrajectoryWarden3D>();
  auto trajectory_warden_out = std::make_shared<TrajectoryWarden3D>();
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    trajectory_warden_in->Register(quad_name);
    trajectory_warden_out->Register(quad_name);
  }

  // For every quad, subscribe to its corresponding proposed_trajectory topic
  std::unordered_map<std::string, std::shared_ptr<TrajectorySubscriberNode3D>> trajectory_subscribers;
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    trajectory_subscribers[quad_name] = 
        std::make_shared<TrajectorySubscriberNode3D>(
            topic, 
            quad_name, 
            trajectory_warden_in);
  }

  // Initialize publishers
  // Load the publisher topics
  std::map<std::string, std::string> updated_trajectory_topics;
  if(false == nh.getParam("updated_trajectory_topics", updated_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: updated_trajectory_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // For every quad, publish to its corresponding updated_trajectory topic
  std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode3D>> trajectory_publishers;
  for(const auto& kv: updated_trajectory_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    trajectory_publishers[quad_name] = 
      std::make_shared<TrajectoryPublisherNode3D>(topic);
  }

  // Initialize the QuadStateWarden. The QuadStateWarden enables safe, multi-threaded
  // access to quadcopter state data. Internal components that require access to
  // state data should request access through QuadStateWarden.
  auto state_warden  = std::make_shared<QuadStateWarden3D>();
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    state_warden->Register(quad_name);
  }

  // For every quad, subscribe to its corresponding state topic
  std::vector<std::shared_ptr<QuadStateSubscriberNode3D>> state_subscribers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;  
    const std::string& topic = kv.second;  
    state_subscribers.push_back(
        std::make_shared<QuadStateSubscriberNode3D>(
            topic, 
            quad_name, 
            state_warden));
  }

  // TrajectoryDispatcher thread. TrajectoryDispatcher pipes data from
  // TrajectoryWarden to its corresponding trajectory publisher. Runs as a
  // separate thread and maintains a thread pool. Each thread in the pool is
  // assigned a particular trajectory and blocks until that trajectory is modified.
  // Once modified, it moves the data into the corresponding publisher queue.
  auto trajectory_dispatcher = std::make_shared<TrajectoryDispatcher3D>();
  std::thread trajectory_dispatcher_thread([&](){
      trajectory_dispatcher->Run(trajectory_warden_out, trajectory_publishers);
      });

  // Mediation layer thread. The mediation layer runs continuously, forward
  // integrating the proposed trajectories and modifying them so that the
  // various agents will not crash into each other. Data is asynchonously read
  // and written from the TrajectoryWardens
  auto mediation_layer = std::make_shared<MediationLayer3D>();
  std::thread mediation_layer_thread(
      [&]() {
        mediation_layer->Run(
            trajectory_warden_in, 
            trajectory_warden_out, 
            state_warden);
      });

  // TODO: WIP
  //    for(const Plane3D face: obstacle.Faces()) {
  //      const auto potential = std::make_shared<Plane3DPotential>(
  //          face, 
  //          environment_view_options.plane3d_potential_options);
  //    }
  std::string quad_mesh_file_path;
  if(false == nh.getParam("quad_mesh_file_path", quad_mesh_file_path)) {
    std::cerr << "Required parameter not found on server: quad_mesh_file_path" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  auto quad_guard = std::make_shared<QuadStateGuard3D>();
  std::vector<std::shared_ptr<Potential3D>> potentials;

  for(const Polyhedron& obstacle: map.Obstacles()) {
    for(const Plane3D face: obstacle.Faces()) {
      potentials.push_back(
          std::make_shared<Plane3DPotential>(
            face, 
            Plane3DPotential::Options()));
    }
  }

  ViewManager3D::QuadViewOptions quad_view_options;
  quad_view_options.quad_mesh_file_path = quad_mesh_file_path;
  quad_view_options.quads.push_back(std::make_pair<>("blue", quad_guard));

  ViewManager3D::EnvironmentViewOptions environment_view_options;
  environment_view_options.map = map;
  environment_view_options.potentials = potentials;

  auto view_manager = std::make_shared<ViewManager3D>();
  std::thread view_manager_thread(
      [&]() {
        view_manager->Run(
            quad_view_options,
            environment_view_options);
      });
  // TODO: END WIP

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

        mediation_layer->Stop();
        view_manager->Stop();
        trajectory_dispatcher->Stop();

        trajectory_warden_in->Stop();
        trajectory_warden_out->Stop();
        state_warden->Stop();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();

  // Wait for other threads to die
  trajectory_dispatcher_thread.join();
  mediation_layer_thread.join();
  view_manager_thread.join();

  return EXIT_SUCCESS;
}

