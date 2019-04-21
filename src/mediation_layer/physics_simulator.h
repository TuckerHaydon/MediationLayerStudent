// Author: Tucker Haydon

#pragma once 

#include <atomic>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <random>
#include <utility>
#include <map>

#include "trajectory_warden.h"
#include "quad_state_publisher_node.h"
#include "runge_kutta_4.h"


namespace mediation_layer {
  namespace{
		/* mvnrnd
    * Generates a multivariate gaussian sample.
    * mu:     gaussian mean
    * sigma:  gaussian covariance 
    */
    Eigen::MatrixXd mvnrnd(const Eigen::VectorXd& mu, 
                           const Eigen::MatrixXd& sigma) {
      // A Mersenne Twister pseudo-random generator of 32-bit numbers with a
      // state size of 19937 bits.
      std::mt19937 gen{ std::random_device{}() };
      
      // Normal distribution
      std::normal_distribution<> dist;
      
      // Find the square root of the covariance matrix. Based off of
      // diagonalization decomposition (VDV'). Multiply V sqrt(D).
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(sigma);
      Eigen::MatrixXd sqrt_sigma = eigen_solver.eigenvectors() * 
        eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();

      // Conjour a multivariate gaussian sample. Generate a vector with three
      // unit gaussian samples. Multiply it by the square root of the
      // covariance matrix and add it to the mean.
      return mu + 
        sqrt_sigma * 
        Eigen::MatrixXd(mu.rows(), mu.cols()).unaryExpr(
          [&](auto x) { return dist(gen); });
    } 

    struct WindInstance {
      // Floating point time in seconds since the epoch
      double time;

      // Acceleration in m/s^2
      Eigen::Vector3d acceleration;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      WindInstance() {}
    };
  }


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
        size_t integration_steps = 10;

        // Proportional constant
        double kp = -1;

        // Derivative constant
        double kd = -0.1;

        // Gauss markov constant
        double alpha = 0.85;

        // Gauss markov noise mean
        Eigen::Vector3d mu = Eigen::Vector3d(0,0,0);

        // Gauss markov noise covariance
        Eigen::Matrix3d sigma = (Eigen::Matrix3d() << 
            5e-4,    0,    0,
            0,    5e-4,    0,
               0,    0, 5e-4).finished();


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

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  void PhysicsSimulator::Run(
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::unordered_map<std::string, std::shared_ptr<QuadStatePublisherNode>> quad_state_publishers) {
    
    // System time
    const auto start_time = std::chrono::system_clock::now();

    // Wind
    Eigen::Vector3d last_wind_acceleration(0,0,0);
    const size_t wind_vector_size 
      = static_cast<size_t>(std::ceil(
            this->options_.simulation_time / 
            this->options_.wind_zoh_time));

    // Integrator
    RungeKutta4<Eigen::Vector<double, 9>> rk4;

    // Extract quad names
    std::vector<std::string> quad_names;
    for(const auto& kv: quad_state_publishers) {
      quad_names.push_back(kv.first);
    }

    // Store the last location of the quads
    std::map<
      std::string, 
      Eigen::Vector<double, 9>, 
      std::less<std::string>, 
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector<double, 9>>>> pva_perturbed_register;

    // Initially, the quads are in their current state
    // TODO: Change origin to state
    for(const std::string& quad_name: quad_names) {
      pva_perturbed_register[quad_name] = Eigen::Vector<double, 9>::Zero();
    }

    while(true == this->ok_) {
      // Current time
      const auto current_time = std::chrono::system_clock::now();

      // Acceleration vector caused by the wind. Using a first-order
      // guass-markov model
      std::vector<WindInstance> wind_instances(wind_vector_size);
      for(size_t wind_idx = 0; wind_idx < wind_vector_size; ++wind_idx) {
        const Eigen::Vector3d noise = mvnrnd(this->options_.mu, this->options_.sigma);
        const Eigen::Vector3d wind_acceleration = this->options_.alpha * last_wind_acceleration + noise;
        WindInstance wind_instance;
        wind_instance.time 
          = std::chrono::duration_cast<std::chrono::duration<double>>
          ((current_time + wind_idx * this->options_.wind_zoh_time)
           .time_since_epoch()).count();
        wind_instance.acceleration = wind_acceleration;

        wind_instances[wind_idx] = wind_instance;
        last_wind_acceleration = wind_acceleration;
      }

      // Forward integrate all quadcopter trajectories, injecting noise. The
      // integration period is specified in the options structure.
      const double current_time_float
          = std::chrono::duration_cast<std::chrono::duration<double>>(
              current_time.time_since_epoch()).count();
      const double end_time_float
          = std::chrono::duration_cast<std::chrono::duration<double>>(
              current_time.time_since_epoch() + this->options_.simulation_time).count();

      for(const std::string& quad_name: quad_names) {
        // Read the most current trajectory
        Trajectory trajectory;
        trajectory_warden_in->Read(quad_name, trajectory);

        // Require a trajectory to be published
        const size_t trajectory_size = trajectory.Size();
        if(trajectory_size == 0) {
          std::cout << "PhysicsSimulator::Run -- Trajectory not specified." << std::endl;
          continue;
        }

        // A trajectory may not have been updated between simulation periods.
        // Must find the index in the trajectory that most closely aligns with
        // the current time. Assuming a zero-order-hold for intended trajectory
        // inputs. Always round down.
        size_t trajectory_idx = 0;
        for(size_t idx = 0; idx < trajectory_size; ++idx) {
          if(trajectory.Time(idx) > current_time_float) {
            trajectory_idx = trajectory_idx == 0 ? 0 : trajectory_idx - 1;
          } else {
            continue;
          }
        }

        // Forward simulate
        // If simulation window extends beyond the provided trajectory window,
        // hold the last position. 
        Eigen::Vector<double, 9> pva_intended = trajectory.PVA(trajectory_idx);
        Eigen::Vector<double, 9> pva_perturbed = pva_perturbed_register[quad_name];
        TimeSpan ts(0,1,0.5);
        while(true) {
          if(trajectory_idx+1 < trajectory_size) {
            // Go to next segment
            pva_intended = trajectory.PVA(trajectory_idx);

            const double t0 = trajectory.Time(trajectory_idx);
            const double tf = trajectory.Time(trajectory_idx + 1);
            const double dt = (tf - t0) / this->options_.integration_steps;
            ts = TimeSpan(t0, tf, dt);
          } else {
            // Hold final position
            pva_intended = (Eigen::Vector<double, 9>() << 
              trajectory.Position(trajectory_size - 1),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()).finished();

            const double t0 = trajectory.Time(trajectory_size - 1);
            const double tf = end_time_float + 0.01;
            const double dt = (tf - t0) / this->options_.integration_steps;
            ts = TimeSpan(t0, tf, dt);
          }

          // Dynamics matrix
          Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Zero();
          A.block(0,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
          A.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * 1 * this->options_.kp;
          A.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * 1 * this->options_.kd;

		      // Input matrix
		      Eigen::Matrix<double, 9, 12> B = Eigen::Matrix<double, 9, 12>::Zero();
          B.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
          B.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * -1 * this->options_.kp;
          B.block(3,6,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * -1 * this->options_.kd;
          B.block(3,9,3,3) = Eigen::Matrix<double, 3, 3>::Identity();

          auto DynamicsFunction = [&](
              double time, 
              const Eigen::Vector<double, 9>& pva_intermediate) {
						WindInstance disturbance_instance;
						for(size_t wind_idx = 0; wind_idx < wind_vector_size; ++wind_idx) {
							WindInstance wind_instance = wind_instances[wind_idx];
							if(wind_instance.time > time) {
								disturbance_instance = wind_instance;
								break;
							}
						}

            Eigen::Vector<double, 12> input;
            input.block(0,0,3,1) = disturbance_instance.acceleration;
            input.block(3,0,9,1) = pva_intended;

            const Eigen::Vector<double, 9> t1 = A * pva_intermediate;
            const Eigen::Vector<double, 9> t2 = B * input;
            const Eigen::Vector<double, 9> t3 = t1 + t2;

            return t3;
          };

          // The pva_perturbed is determined by forward integrating the dynamics
          // from the previous pva_perturbed
          pva_perturbed = rk4.ForwardIntegrate(DynamicsFunction, pva_perturbed, ts);  

          trajectory_idx++;

          if(ts.tf_ > end_time_float) {
            break;
          }
        }

        pva_perturbed_register[quad_name] = pva_perturbed;
      }

      std::this_thread::sleep_until(current_time + this->options_.simulation_time);

      // Publish
      for(const auto& kv: quad_state_publishers) {
        const std::string& quad_name = kv.first;
        const Eigen::Vector<double, 9> pva_perturbed = pva_perturbed_register[quad_name];
        QuadState quad_state(Eigen::Vector<double, 13>(
              pva_perturbed(0), pva_perturbed(1), pva_perturbed(2),
              pva_perturbed(3), pva_perturbed(4), pva_perturbed(5),
              1,0,0,0,
              0,0,0 
              ));
        quad_state_publishers[quad_name]->Publish(quad_state);
      }
    }

  }

  inline void PhysicsSimulator::Stop() {
    this->ok_ = false;
  }
}
