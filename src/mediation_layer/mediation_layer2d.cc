// Author: Tucker Haydon

#include <chrono>
#include <thread>

#include "mediation_layer2d.h"
#include "line2d_force.h"
#include "runge_kutta_4.h"
#include "time_span.h"
#include "vec2d.h"
#include "timer.h"

namespace path_planning {
  using Trajectory_t = Eigen::Matrix<double, 6, 1>;

  namespace {
    Trajectory_t DynamicsFun(const double time, 
                                            const Trajectory_t& X_mediation_layer, 
                                            const Trajectory_t& X_reference,
                                            const Vec2D& force) {
      static const double kp = -1;
      static const double kd = -1;

      const double x_ml  = X_mediation_layer(0);
      const double y_ml  = X_mediation_layer(1);
      const double vx_ml = X_mediation_layer(2);
      const double vy_ml = X_mediation_layer(3);
      const double ax_ml = X_mediation_layer(4);
      const double ay_ml = X_mediation_layer(5);

      const double x_ref  = X_reference(0);
      const double y_ref  = X_reference(1);
      const double vx_ref = X_reference(2);
      const double vy_ref = X_reference(3);
      const double ax_ref = X_reference(4);
      const double ay_ref = X_reference(5);


      return (Trajectory_t() << 
          vx_ml, 
          vy_ml, 
          ax_ref + kp*(x_ml - x_ref) + kd*(vx_ml - vx_ref) + force.x(), 
          ay_ref + kp*(y_ml - y_ref) + kd*(vy_ml - vy_ref) + force.y(), 
          0, 
          0).finished();
    }

    Trajectory2D GenerateSmoothTrajectory(
        const Trajectory_t& start, 
        const Trajectory_t& end, 
        const double dt,
        const double time) {
      const Eigen::Matrix<double, 6, 6> state_transition_matrix = 
        (Eigen::Matrix<double, 6, 6>() << 
         1, 0, dt, 0   dt*dt/2, 0,
         0, 1, 0,  dt, 0,       dt*dt/2,
         0, 0, 1,  0,  dt,      0,
         0, 0, 0,  1,  0,       dt,
         0, 0, 0,  0,  1,       0,
         0, 0, 0,  0,  0,       1
         ).finished();

      const int N = (int)(time / dt) + 1;
      std::vector<TimeStampedPVAY2D> tspvay2D;
      tspvay2D.reserve(N);
      for(int idx = 0; idx < N; ++idx) {

      }
    }
  }
  
  bool MediationLayer2D::Run() {
    /* ALGORITHM
      for every trajectory in proposed_state
         Forward integrate mediation layer dynamics
         Write resulting trajectory to updated_state
    */

    const Point2D a(-0.6, -0.5), b(-0.4, -0.5);
    const Line2D line(a,b);
    Line2DForce::Options options;
    options.activation_dist = 0.8;
    options.min_dist = 0.1;
    options.scale = 0.1;
    const Line2DForce force_field(line, options);

    const std::string& key = "test";
    Trajectory2D proposed_trajectory({
        TimeStampedPVAY2D(PVAY2D(Vec2D(-1.0,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.0),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.9,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.1),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.8,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.2),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.7,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.3),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.6,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.4),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.5,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.5),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.4,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.6),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.3,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.7),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.2,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.8),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.1,0), Vec2D(1,0), Vec2D(0,0), 0.0), 0.9),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.0),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.1),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.2),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.3),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.4),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.5),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.6),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.7),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.8),
        TimeStampedPVAY2D(PVAY2D(Vec2D(-0.0,0), Vec2D(0,0), Vec2D(0,0), 0.0), 1.9)
        });

    this->proposed_state_->Add(key, proposed_trajectory);
    this->updated_state_->Add(key, proposed_trajectory);

    const double t0{0}, tf{0.1}, dt{0.01};
    const TimeSpan ts(t0, tf, dt);
    const RungeKutta4<Trajectory_t> rk4;

    const Trajectory_t X0 = (Trajectory_t() <<
      proposed_trajectory.data_[0].pvay_.position_,
      proposed_trajectory.data_[0].pvay_.velocity_,
      proposed_trajectory.data_[0].pvay_.acceleration_
      ).finished();

    Trajectory_t X_ml = X0;
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    Timer timer("Mediation Layer Timer Terminated.");
    timer.Start();
    int idx = 0;
    while(true) {
      if(idx == proposed_trajectory.data_.size()) { break; }

      const Trajectory_t X_reference = (Trajectory_t() <<
        proposed_trajectory.data_[idx].pvay_.position_,
        proposed_trajectory.data_[idx].pvay_.velocity_,
        proposed_trajectory.data_[idx].pvay_.acceleration_
        ).finished();

      auto rk4_fun = [&](const double time, 
                         const Trajectory_t& X_mediation_layer) {
        return DynamicsFun(
            time, 
            X_mediation_layer, 
            X_reference, 
            force_field.Resolve(Point2D(X_mediation_layer(0), X_mediation_layer(1))));
      };

      X_ml = rk4.ForwardIntegrate(rk4_fun, X_ml, ts);
      std::cout << X_ml.transpose().format(CleanFmt) << std::endl;

      // this->proposed_state_->Read(key, proposed_trajectory);
      // this->updated_state_->Write(key, trajectory);

      idx++;
    }

    timer.Stop();

    return true;
  }
}
