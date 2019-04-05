// Author: Tucker Haydon

#include <cstdlib>
#include <string>
#include <vector>
#include <memory>
#include <csignal>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "game_snapshot.h"
#include "trajectory_warden.h"
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
  ros::init(argc, argv, "automation_protocol", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/mediation_layer/");

  // YAML config
  std::map<std::string, std::string> team_assignments;
  if(false == nh.getParam("team_assignments", team_assignments)) {
    std::cerr << "Required parameter not found on server: team_assignments" << std::endl;
    std::exit(1);
  }

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

  auto trajectory_warden_in = std::make_shared<TrajectoryWarden3D>();
  auto trajectory_warden_out = std::make_shared<TrajectoryWarden3D>();

  auto game_snapshot = std::make_shared<GameSnapshot3D>(
      blue_quad_names,
      red_quad_names,
      trajectory_warden_in,
      GameSnapshot3D::Options());

  std::shared_ptr<AutomationProtocol3D> automation_protocol = std::make_shared<TestAP>(
      blue_quad_names, 
      red_quad_names,
      game_snapshot,
      trajectory_warden_out);

  std::thread ap_thread(
      [&]() {
        automation_protocol->Run();
      });
  
  std::thread kill_thread(
      [&]() {
        while(true) {
          if(true == kill_program) {
            break;
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          }
        }
        automation_protocol->Stop();
      });

  // Wait for program termination via ctl-c
  kill_thread.join();
  ap_thread.join();

  return EXIT_SUCCESS;
}
