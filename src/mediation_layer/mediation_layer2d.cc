// Author: Tucker Haydon

#include <chrono>
#include <thread>

#include "mediation_layer2d.h"
#include "runge_kutta_4.h"
#include "time_span.h"
#include "timer.h"

using PVA_t = Eigen::Matrix<double, 6, 1>;

namespace mediation_layer { 
  namespace {
    PVA_t DynamicsFun(const double time, 
                             const PVA_t& X_ml, 
                             const PVA_t& X_ref,
                             const Vec2D& force) {
      static const double kp = -3;
      static const double kd = -1.5;

      const double x_ml  = X_ml(0);
      const double y_ml  = X_ml(1);
      const double vx_ml = X_ml(2);
      const double vy_ml = X_ml(3);
      const double ax_ml = X_ml(4);
      const double ay_ml = X_ml(5);

      const double x_ref  = X_ref(0);
      const double y_ref  = X_ref(1);
      const double vx_ref = X_ref(2);
      const double vy_ref = X_ref(3);
      const double ax_ref = X_ref(4);
      const double ay_ref = X_ref(5);

      return (PVA_t() << 
          vx_ml, 
          vy_ml, 
          ax_ref + kp*(x_ml - x_ref) + kd*(vx_ml - vx_ref) + force.x(), 
          ay_ref + kp*(y_ml - y_ref) + kd*(vy_ml - vy_ref) + force.y(), 
          0, 
          0).finished();
    }
  }

  bool MediationLayer2D::Run(
      std::shared_ptr<State2D> proposed_state,
      std::shared_ptr<State2D> updated_state) {

    // 6D RK4
    const RungeKutta4<PVA_t> rk4;

    while(true == this->ok_) {
      for(const std::string& key: proposed_state->Keys()) {
        Trajectory2D proposed_trajectory, updated_trajectory;
        proposed_state->Read(key, proposed_trajectory);

        updated_trajectory.Append(proposed_trajectory.PVAYT(0));
        PVA_t x_ml = proposed_trajectory.PVA(0);
        for(size_t idx = 0; idx < proposed_trajectory.Size()-1; ++idx) {
          // Time span
          const double t0{proposed_trajectory.Time(idx)};
          const double tf{proposed_trajectory.Time(idx+1)};
          const double dt{0.01};
          const TimeSpan ts(0, tf, dt);

          // RK4 function wrapper
          auto rk4_fun = [&](const double time, 
                             const PVA_t& X_ml) {

            const Point2D point(X_ml(0), X_ml(1));

            // const Vec2D resolved_force = std::accumulate(
            //     potentials.begin(),
            //     potentials.end(),
            //     Vec2D(0,0),
            //     [&](const Vec2D& sum, const std::shared_ptr<Potential2D>& potential) {
            //       return sum + potential->Resolve(point);
            //     });
            Vec2D resolved_force;

            return DynamicsFun(
                time, 
                X_ml, 
                proposed_trajectory.PVA(idx),
                resolved_force);
          };

          // Forward integrate
          x_ml = rk4.ForwardIntegrate(rk4_fun, x_ml, ts);

          // Append new instance
          Eigen::Matrix<double, 8, 1> updated_instance 
            = (Eigen::Matrix<double, 8, 1>() <<
            x_ml,
            proposed_trajectory.Yaw(idx+1),
            proposed_trajectory.Time(idx+1)).finished();
          updated_trajectory.Append(updated_instance);
        }
       
        // Write output
        updated_state->Write(key, updated_trajectory);
      }
    }


    return true;
  }

  bool MediationLayer2D::Stop() {
    this->ok_ = false;
    return true;
  }
}
