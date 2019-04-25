// Author: Tucker Haydon

#include "physics_simulator.h"
#include "runge_kutta_4.h"

namespace game_engine {
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
    RungeKutta4<Eigen::Matrix<double, 9, 1>> rk4;

    // Extract quad names
    std::vector<std::string> quad_names;
    for(const auto& kv: quad_state_publishers) {
      quad_names.push_back(kv.first);
    }

    // Store the last location of the quads
    std::map<
      std::string, 
      Eigen::Matrix<double, 9, 1>, 
      std::less<std::string>, 
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix<double, 9, 1>>>> pva_perturbed_register;

    // Initially, the quads are in their current state
    for(const std::string& quad_name: quad_names) {
      pva_perturbed_register[quad_name] = Eigen::Matrix<double, 9, 1>::Zero();
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
          // std::cout << "PhysicsSimulator::Run -- Trajectory not specified." << std::endl;
          continue;
        }

        // A trajectory may not have been updated between simulation periods.
        // Must find the index in the trajectory that most closely aligns with
        // the current time. Assuming a zero-order-hold for intended trajectory
        // inputs. Always round down.
        size_t trajectory_idx = 0;
        if(trajectory.Time(trajectory_size-1) < current_time_float) {
          trajectory_idx = trajectory.Size()-1;
        } else {
          for(size_t idx = 0; idx < trajectory_size; ++idx) {
            if(trajectory.Time(idx) > current_time_float) {
              trajectory_idx = ( (idx == 0) ? 0 : idx - 1 );
              break;
            }
          }
        }

        // Forward simulate
        // If simulation window extends beyond the provided trajectory window,
        // hold the last position. 
        Eigen::Matrix<double, 9, 1> pva_intended = trajectory.PVA(trajectory_idx);
        Eigen::Matrix<double, 9, 1> pva_perturbed = pva_perturbed_register[quad_name];
        TimeSpan ts(0,1,0.5);
        while(true) {
          if(trajectory_idx != trajectory_size - 1) {
            // Go to next segment
            pva_intended = trajectory.PVA(trajectory_idx);

            const double t0 = trajectory.Time(trajectory_idx);
            const double tf = trajectory.Time(trajectory_idx + 1);
            const double dt = (tf - t0) / this->options_.integration_steps;
            ts = TimeSpan(t0, tf, dt);
          } else {
            // Hold final position
            pva_intended = (Eigen::Matrix<double, 9, 1>() << 
              trajectory.Position(trajectory_size - 1),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()).finished();

            const double t0 = current_time_float;
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
              const Eigen::Matrix<double, 9, 1>& pva_intermediate) {
						WindInstance disturbance_instance;
						for(size_t wind_idx = 0; wind_idx < wind_vector_size; ++wind_idx) {
							WindInstance wind_instance = wind_instances[wind_idx];
							if(wind_instance.time > time) {
								disturbance_instance = wind_instance;
								break;
							}
						}

            Eigen::Matrix<double, 12, 1> input = Eigen::Matrix<double, 12, 1>::Zero();
            // input.block(0,0,3,1) = disturbance_instance.acceleration;
            input.block(3,0,9,1) = pva_intended;

            const Eigen::Matrix<double, 9, 1> t1 = A * pva_intermediate;
            const Eigen::Matrix<double, 9, 1> t2 = B * input;
            const Eigen::Matrix<double, 9, 1> t3 = t1 + t2;

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
        const Eigen::Matrix<double, 9, 1> pva_perturbed = pva_perturbed_register[quad_name];
        QuadState quad_state(Eigen::Matrix<double, 13, 1>(
              pva_perturbed(0), pva_perturbed(1), pva_perturbed(2),
              pva_perturbed(3), pva_perturbed(4), pva_perturbed(5),
              1,0,0,0,
              0,0,0 
              ));
        quad_state_publishers[quad_name]->Publish(quad_state);
      }
    }

  }

  void PhysicsSimulator::Stop() {
    this->ok_ = false;
  }
}
