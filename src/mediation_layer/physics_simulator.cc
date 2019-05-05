// Author: Tucker Haydon

#include "physics_simulator.h"
#include "runge_kutta_4.h"

#include <fftw3.h>

namespace game_engine {
  namespace{
    // Generates a multivariate gaussian sample.
    // mu:     mean
    // sigma:  covariance 
    // gen:    random number generator
    Eigen::VectorXd mvnrnd(
        const Eigen::VectorXd& mu, 
        const Eigen::MatrixXd& sigma,
        std::mt19937& gen) {
      
      // Normal distribution
      std::normal_distribution<double> dist(0.0, 1.0);
      
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

    // Samples a VonKarman PSD at a given spatial frequency, Omega
    // Reference: 
    // https://en.wikipedia.org/wiki/Von_Kármán_wind_turbulence_model
    Eigen::VectorXd VonKarman(
        const Eigen::VectorXd& Omega, 
        const double& sigma_u, 
        const double& L_u) {
      return
        (2 * L_u * std::pow(sigma_u, 2) / M_PI) *
        (1 + (1.339 * L_u * Omega.array()).pow(2)).pow(-(5.0/6.0));
    }

    struct WindInstance {
      // Floating point time in seconds since the epoch
      double time;

      // Acceleration in m/s^2
      Eigen::Vector3d acceleration;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      WindInstance() {}
    };

    // Returns a time-history of wind-acceleration generated to have have a
    // VonKarman PSD. Returns a pair of vectors <time, acceleration>
    std::pair<std::vector<double>, std::vector<double>> GenerateWindAccelerationVector(
        const double max_time,
        const double max_frequency,
        const double sigma_u,
        const double L_u,
        const double V,
        std::mt19937& gen) {

      // Number of samples
      const size_t N = std::floor(max_time * max_frequency) + 1;
      // Frequency spacing
      const double df = max_frequency / N;

      // Vector of frequencies to sample VonKarman PSD at
      Eigen::VectorXd Omega_vec;
      Omega_vec.resize(N);
      for(size_t idx = 0; idx < N; ++idx) {
        Omega_vec(idx,0) = (2 * M_PI / V) * (idx * df);
      }

      // Sample the VonKarman distribution
      const Eigen::VectorXd S = VonKarman(Omega_vec, sigma_u, L_u);

      // Prepare for fft
      Eigen::VectorXcd noise_freq;
      noise_freq.resize(N);
      {
        fftw_complex *in, *out;
        fftw_plan p;
        in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
        out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
        p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

        // Calculate white noise 
        for(size_t idx = 0; idx < N; ++idx) {
          Eigen::VectorXd mu;
          mu.resize(N,1);
          mu.fill(0);

          Eigen::MatrixXd sigma;
          sigma.resize(N,N);
          sigma.setIdentity();

          in[idx][0] =  mvnrnd(Eigen::Matrix<double,1,1>(0), Eigen::Matrix<double,1,1>::Identity(), gen)(0);
          in[idx][1] =  mvnrnd(Eigen::Matrix<double,1,1>(0), Eigen::Matrix<double,1,1>::Identity(), gen)(0);
        }

        // FFT of white noise
        fftw_execute(p);
        fftw_destroy_plan(p);

        // Copy FFT to Eigen
        for(size_t idx = 0; idx < N; ++idx) {
          noise_freq(idx,0).real(out[idx][0]);
          noise_freq(idx,0).imag(out[idx][1]);
        }

        // Free fft
        fftw_free(in); 
        fftw_free(out);
      }

      // Transform PSD samples to amplitude
      const Eigen::VectorXcd A = S.array().sqrt();

      // Push white noise through amplitude filter
      const Eigen::VectorXcd wind_freq 
        = A.cwiseProduct(noise_freq);

      // Take the inverse fourier transform
      Eigen::VectorXcd wind_complex_time;
      wind_complex_time.resize(N);
      {
        fftw_complex *in, *out;
        fftw_plan p;
        in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
        out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
        p = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

        // Copy wind vector into fftw buffer
        for(size_t idx = 0; idx < N; ++idx) {
          in[idx][0] = wind_freq(idx,0).real();
          in[idx][1] = wind_freq(idx,0).imag();
        }

        // FFT
        fftw_execute(p);
        fftw_destroy_plan(p);

        // Copy FFT to Eigen
        for(size_t idx = 0; idx < N; ++idx) {
          wind_complex_time(idx,0).real(out[idx][0]);
          wind_complex_time(idx,0).imag(out[idx][1]);
        }

        // Divide by N to normalize since FFTW does not normalize
        wind_complex_time = wind_complex_time/N;

        // Free fft
        fftw_free(in); 
        fftw_free(out);
      }

      // Extract real parts and scale
      Eigen::VectorXd wind_time;
      wind_time.resize(N);
      for(size_t idx = 0; idx < N; ++idx) {
        wind_time(idx,0) = std::sqrt(2) * wind_complex_time(idx,0).real();
      }


      // Push into std::vector
      std::vector<double> time_vec;
      std::vector<double> wind_vec;
      time_vec.reserve(N);
      wind_vec.reserve(N);

      for(size_t idx = 0; idx < N; ++idx) {
        time_vec.push_back(max_time/N * idx);
        wind_vec.push_back(wind_time(idx,0));
      }

      return std::make_pair(time_vec, wind_vec);
    }
  }

  void PhysicsSimulator::Run(
      std::shared_ptr<TrajectoryWarden> trajectory_warden_in,
      std::unordered_map<std::string, std::shared_ptr<QuadStatePublisherNode>> quad_state_publishers) {
    
    // System time
    const auto start_time = std::chrono::system_clock::now();

    // Generate wind
    std::mt19937 gen{ std::random_device{}() };
    // std::mt19937 gen{ 0 };

    const auto x_wind_samples = GenerateWindAccelerationVector(
        this->options_.max_time,
        this->options_.max_frequency,
        this->options_.sigma_u_x,
        this->options_.L_u_x,
        this->options_.V,
        gen);

    const auto y_wind_samples = GenerateWindAccelerationVector(
        this->options_.max_time,
        this->options_.max_frequency,
        this->options_.sigma_u_y,
        this->options_.L_u_y,
        this->options_.V,
        gen);

    const auto z_wind_samples = GenerateWindAccelerationVector(
        this->options_.max_time,
        this->options_.max_frequency,
        this->options_.sigma_u_z,
        this->options_.L_u_z,
        this->options_.V,
        gen);

    const size_t wind_vector_size = x_wind_samples.first.size();

    std::vector<WindInstance> wind_instances;
    for(size_t wind_idx = 0; wind_idx < wind_vector_size; ++wind_idx) {
      WindInstance wind_instance;
      wind_instance.time = 
        std::chrono::duration_cast<std::chrono::duration<double>>(start_time.time_since_epoch()).count()
        + x_wind_samples.first[wind_idx];
      wind_instance.acceleration = Eigen::Vector3d(
        x_wind_samples.second[wind_idx],
        y_wind_samples.second[wind_idx],
        z_wind_samples.second[wind_idx]
          );

      wind_instances.push_back(wind_instance);
    }

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
      pva_perturbed_register[quad_name] = (Eigen::Matrix<double, 9, 1>() <<
          this->options_.initial_quad_positions[quad_name](0),
          this->options_.initial_quad_positions[quad_name](1),
          this->options_.initial_quad_positions[quad_name](2),
          0,0,0,
          0,0,0
            ).finished();
    }

    while(true == this->ok_) {
      // Current time
      const auto current_time = std::chrono::system_clock::now();

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
          // State is: [x, x_dot, x_ddot]', x is the physics state
          //| v_x |    | 0.00 0.00 0.00 1.00 0.00 0.00 0.00 0.00 0.00 | | x_x |
          //| v_y |    | 0.00 0.00 0.00 0.00 1.00 0.00 0.00 0.00 0.00 | | x_y |
          //| v_z |    | 0.00 0.00 0.00 0.00 0.00 1.00 0.00 0.00 0.00 | | x_z |
          //| a_x |    |   kp 0.00 0.00   kd 0.00 0.00 0.00 0.00 0.00 | | v_x |
          //| a_y | =  | 0.00   kp 0.00 0.00   kd 0.00 0.00 0.00 0.00 | | v_y |
          //| a_z |    | 0.00 0.00   kp 0.00 0.00   kd 0.00 0.00 0.00 | | v_z |
          //| j_x |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | a_x |
          //| j_y |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | a_y |
          //| j_z |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | a_z |
          Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Zero();
          A.block(0,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
          A.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * 1 * this->options_.kp;
          A.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * 1 * this->options_.kd;

		      // Input matrices
          // Input is: [x, x_dot, x_ddot]', x is the intended state
          //| v_x |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | x_x |
          //| v_y |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | x_y |
          //| v_z |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | x_z |
          //| a_x |    |  -kp 0.00 0.00  -kd 0.00 0.00 1.00 0.00 0.00 | | v_x |
          //| a_y | =  | 0.00  -kp 0.00 0.00  -kd 0.00 0.00 1.00 0.00 | | v_y |
          //| a_z |    | 0.00 0.00  -kp 0.00 0.00  -kd 0.00 0.00 1.00 | | v_z |
          //| j_x |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | a_x |
          //| j_y |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | a_y |
          //| j_z |    | 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 | | a_z |
		      Eigen::Matrix<double, 9, 9> B1 = Eigen::Matrix<double, 9, 9>::Zero();
          B1.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * -1 * this->options_.kp;
          B1.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity() * -1 * this->options_.kd;
          B1.block(3,6,3,3) = Eigen::Matrix<double, 3, 3>::Identity();

          // Input is: [F]'
          //| v_x |    | 0.00 0.00 0.00 | | F_x |
          //| v_y |    | 0.00 0.00 0.00 | | F_y |
          //| v_z |    | 0.00 0.00 0.00 | | F_z |
          //| a_x |    | 1.00 0.00 0.00 |
          //| a_y | =  | 0.00 1.00 0.00 |
          //| a_z |    | 0.00 0.00 1.00 |
          //| j_x |    | 0.00 0.00 0.00 |
          //| j_y |    | 0.00 0.00 0.00 |
          //| j_z |    | 0.00 0.00 0.00 |
          Eigen::Matrix<double, 9, 3> B2 = Eigen::Matrix<double, 9, 3>::Zero();
          B2.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();

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

            // Input constrain the max acceleration of the PD loop
            Eigen::Matrix<double, 9, 1> t1 = A * pva_intermediate + B1 * pva_intended;
            t1.block(3,0,3,1) = t1.block(3,0,3,1).unaryExpr([](double x){
               if(x < -0.8) { return -0.8; }
               if(x >  0.8) { return  0.8; }
               return x;
                }).cast<double>();

            Eigen::Vector3d input = disturbance_instance.acceleration;
            // Eigen::Vector3d input = Eigen::Vector3d::Zero();
            const Eigen::Matrix<double, 9, 1> t2 = B2 * input;
            // std::cout << t1.block(3,0,3,1).transpose() << std::endl;
            // std::cout << t2.block(3,0,3,1).transpose() << std::endl;
            // std::cout << std::endl;

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
