// Author: Tucker Haydon

#include <cstdlib>
#include <string>
#include <vector>
#include <memory>

#include "game_snapshot.h"
#include "trajectory_warden.h"

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

  // GameSnapshot3D red_snapshot(
  //   const std::vector<std::string> friendly_names,
  //   const std::vector<std::string> enemy_names,
  //   const std::shared_ptr<TrajectoryWarden<T>> trajectory_warden_in,
  //   const GameSnapshot3D::Options())

  return EXIT_SUCCESS;
}
