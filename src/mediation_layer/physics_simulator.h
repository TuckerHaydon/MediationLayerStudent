// Author: Tucker Haydon

#pragma once 

#include <atomic>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <random>
#include <thread>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <utility>
#include <map>

#include "trajectory_warden.h"
#include "quad_state_publisher_node.h"

namespace game_engine {
  // The physics simulator alters the path of mediated trajectories to create
  // stochastic variations in a manner that mirrors conditions seen in real
  // life. After introducing the variations, the simulator selects the perturbed state of
  // the quadcopter some time in the future and publishes that as the 'current
  // state' of the quad.
  //
  // The model used for the physics simulator is described in the following
  // section. x_m denotes the mediated (intended) trajectory and x_p denotes the
  // perturbed trajectory. x_p is solved by forward-integrating the intended
  // trajectory with the following model:
  //   x_ddot_p = x_ddot_m + kd * (x_dot_p - x_dot_m) + kp * (x_p - x_m) + F
  // F is a catch-all force introduced to model controllability constraints,
  // wind, and other stochastic forces that prevent the quadcopters from
  // perfectly following the intended trajectory.
  //
  // In the presence of no forcing function, the perturbed trajectory will be
  // exactly the mediated trajectory (provided the mediation trajectory is
  // smooth).
  //
  // TODO: Change the starting location of the quadcopter
  class PhysicsSimulator {
    public:
      struct Options {
        // Time in seconds in which to forward-simulate
        std::chrono::microseconds simulation_time = std::chrono::milliseconds(20);

        // Number of intermediate RK4 integration steps
        size_t integration_steps = 20;

        // From linear system analysis, kp < 0, kd < 0, kp < -0.25 * kd^2
        // Proportional constant
        // double kp = -25.0;
        double kp = -1.0;

        // Derivative constant
        // double kd = -10.0;
        double kd = -2.0;

        // Time in seconds to calculate wind history
        double max_time = 300.0;

        // Frequency to sample wind history
        double max_frequency = 1.0;

        // Standard deviation of VonKarman model
        // double sigma_u_x = 0.3;
        // double sigma_u_y = 0.3;
        // double sigma_u_z = 0.2;
        double sigma_u_x = 0.1;
        double sigma_u_y = 0.1;
        double sigma_u_z = 0.05;

        // Scale length of VonKarman model
        double L_u_x = 1.5;
        double L_u_y = 1.5;
        double L_u_z = 1.5;

        // Quad speed for VonKarman model
        double V = 1.0;

        std::map<
          std::string, 
          Eigen::Vector3d, 
          std::less<std::string>, 
          Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> initial_quad_positions;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Options() {}
      };

      PhysicsSimulator(
          const Options& options)
        : options_(options) {}

      void Run(
          std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
          std::unordered_map<std::string, std::shared_ptr<QuadStatePublisherNode>> quad_state_publishers);

      void Stop();

    private:
      Options options_;
      volatile std::atomic_bool ok_{true};

  };
}
