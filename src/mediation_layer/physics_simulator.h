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

namespace mediation_layer {
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

        // Time in ms over which the wind is assumed constant. Must
        // determine a new wind force every wind_zoh_time seconds. In effect,
        // must allocate a vector of size simulation_time/wind_zoh_time, precaclulate the
        // wind acceleration based on a model, and apply it to every quadcopter
        std::chrono::microseconds wind_zoh_time = std::chrono::milliseconds(1);

        // Number of intermediate RK4 integration steps
        size_t integration_steps = 200;

        // Proportional constant
        double kp = -25.00;

        // Derivative constant
        double kd = -10.0;

        // Gauss markov constant
        double alpha = 0.85;

        // Gauss markov noise mean
        Eigen::Vector3d mu = Eigen::Vector3d(0,0,0);

        // Gauss markov noise covariance
        Eigen::Matrix3d sigma = (Eigen::Matrix3d() << 
         3e-0,     0,      0,
            0,  3e-0,     0,
            0,     0,   2e-0).finished();


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
