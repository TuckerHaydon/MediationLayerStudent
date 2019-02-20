// Author: Tucker Haydon

#include <chrono>
#include <thread>

#include "mediation_layer2d.h"
#include "runge_kutta_4.h"
#include "time_span.h"
#include "timer.h"
#include "trajectory2d_view.h"
#include "line2d_potential.h"
#include "line2d_potential_view.h"
#include "point2d_potential.h"
#include "point2d_potential_view.h"
#include "polygon_potential.h"
#include "polygon_potential_view.h"

namespace mediation_layer {
  using Trajectory_t = Eigen::Matrix<double, 6, 1>;

  namespace {
    Trajectory_t DynamicsFun(const double time, 
                                            const Trajectory_t& X_mediation_layer, 
                                            const Trajectory_t& X_reference,
                                            const Vec2D& force) {
      static const double kp = -3;
      static const double kd = -1.5;

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
        const TimeStampedPVAY2D& start, 
        const double time,
        const double dt) {
      const double e_dt = std::exp(-dt); 
      const Eigen::Matrix<double, 6, 6> state_transition_matrix = 
        (Eigen::Matrix<double, 6, 6>() << 
         e_dt, 0, 0, 0, 0, 0,
         0, e_dt, 0, 0, 0, 0,
         0, 0, e_dt, 0, 0, 0,
         0, 0, 0, e_dt, 0, 0,
         0, 0, 0, 0, e_dt, 0,
         0, 0, 0, 0, 0, e_dt
         ).finished();

      const int N = (int)(time / dt) + 1;
      std::vector<TimeStampedPVAY2D> tspvay2D;
      tspvay2D.reserve(N);
      tspvay2D.push_back(start);
      for(int idx = 0; idx < N-1; ++idx) {
        const Eigen::Matrix<double, 6, 1> current = (Eigen::Matrix<double, 6, 1>() <<
          tspvay2D.back().pvay_.position_,
          tspvay2D.back().pvay_.velocity_,
          tspvay2D.back().pvay_.acceleration_).finished();
        const Eigen::Matrix<double, 6, 1> next = state_transition_matrix * current;

        tspvay2D.emplace_back(
            PVAY2D(
              Vec2D(next(0), next(1)),
              Vec2D(next(2), next(3)),
              Vec2D(next(4), next(5)),
              0),
            tspvay2D.back().time_ + dt);
      }
      return Trajectory2D(tspvay2D);
    }
  }
  
  bool MediationLayer2D::Run(
      State2D& proposed_state,
      State2D& updated_state) {
    /* ALGORITHM
      for every trajectory in proposed_state
         Forward integrate mediation layer dynamics
         Write resulting trajectory to updated_state
    */

    // Line above pushing down, line below pushing up
    const Point2D a(-5, -1), b(2, -1), c(2, 1), d(-5, 1), e(-4, 0.3), f(-0.5, -0.1), g(-0.5, 0.8);
    const Line2D l1(a,b), l2(c,d);
    Polygon poly;
    poly.ConstructFromPoints({
        Point2D(-1.5,-1), 
        Point2D(-1.0,-1),
        Point2D(-1.0,-0.3),
        Point2D(-1.5,-0.3),
        });
    poly = poly.Invert();

    Line2DPotential::Options line_options;
    line_options.activation_dist = 0.8;
    line_options.min_dist = 0.1;
    line_options.scale = 0.1;

    Point2DPotential::Options point_options;
    point_options.activation_dist = 0.8;
    point_options.min_dist = 0.1;
    point_options.scale = 0.1;

    PolygonPotential::Options poly_options;
    poly_options.activation_dist = 0.8;
    poly_options.min_dist = 0.1;
    poly_options.scale = 0.1;

    std::vector<std::shared_ptr<Potential2D>> potentials;
    auto e_pot = std::make_shared<Point2DPotential>(e, point_options);
    auto f_pot = std::make_shared<Point2DPotential>(f, point_options);
    auto g_pot = std::make_shared<Point2DPotential>(g, point_options);
    auto l1_pot = std::make_shared<Line2DPotential>(l1, line_options);
    auto l2_pot = std::make_shared<Line2DPotential>(l2, line_options);
    auto poly_pot = std::make_shared<PolygonPotential>(poly,poly_options);
    auto map_pot = std::make_shared<PolygonPotential>(this->map_.Boundary(), poly_options);

    // potentials.push_back(l1_pot);
    // potentials.push_back(l2_pot);
    // potentials.push_back(e_pot);
    // potentials.push_back(f_pot);
    // potentials.push_back(g_pot);
    potentials.push_back(poly_pot);
    potentials.push_back(map_pot);


    std::vector<std::shared_ptr<PotentialView>> potential_views;
    // potential_views.push_back(std::make_shared<Point2DPotentialView>(e_pot));
    // potential_views.push_back(std::make_shared<Point2DPotentialView>(f_pot));
    // potential_views.push_back(std::make_shared<Point2DPotentialView>(g_pot));
    // potential_views.push_back(std::make_shared<Line2DPotentialView>(l1_pot));
    // potential_views.push_back(std::make_shared<Line2DPotentialView>(l2_pot));
    potential_views.push_back(std::make_shared<PolygonPotentialView>(poly_pot));
    potential_views.push_back(std::make_shared<PolygonPotentialView>(map_pot));

    const std::string& key = "test";
     Trajectory2D proposed_trajectory = GenerateSmoothTrajectory(
         TimeStampedPVAY2D(
           PVAY2D(
             Vec2D(-5,0),
             Vec2D(0,0),
             Vec2D(0,0),
             0),
           0),10,0.1);

    proposed_state.Add(key, proposed_trajectory);
    updated_state.Add(key, proposed_trajectory);

    const double t0{0}, tf{0.1}, dt{0.01};
    const TimeSpan ts(t0, tf, dt);
    const RungeKutta4<Trajectory_t> rk4;

    const Trajectory_t X0 = (Trajectory_t() <<
      proposed_trajectory.data_[0].pvay_.position_,
      proposed_trajectory.data_[0].pvay_.velocity_,
      proposed_trajectory.data_[0].pvay_.acceleration_
      ).finished();

    Trajectory_t X_ml = X0;

    Timer timer("Mediation Layer Timer Terminated.");
    timer.Start();
    int idx = 0;
    std::vector<TimeStampedPVAY2D> updated_trajectory_hist;
    updated_trajectory_hist.reserve(proposed_trajectory.data_.size());
    updated_trajectory_hist.emplace_back(
        PVAY2D(
          Vec2D(X_ml(0), X_ml(1)),
          Vec2D(X_ml(2), X_ml(3)),
          Vec2D(X_ml(4), X_ml(5)),
          0),
        proposed_trajectory.data_[idx].time_);
    while(true == this->ok_) {

      const Trajectory_t X_reference = (Trajectory_t() <<
        proposed_trajectory.data_[idx].pvay_.position_,
        proposed_trajectory.data_[idx].pvay_.velocity_,
        proposed_trajectory.data_[idx].pvay_.acceleration_
        ).finished();

      auto rk4_fun = [&](const double time, 
                         const Trajectory_t& X_mediation_layer) {

        const Point2D point(X_mediation_layer(0), X_mediation_layer(1));

        const Vec2D resolved_force = std::accumulate(
            potentials.begin(),
            potentials.end(),
            Vec2D(0,0),
            [&](const Vec2D& sum, const std::shared_ptr<Potential2D>& potential) {
              return sum + potential->Resolve(point);
            });

        return DynamicsFun(
            time, 
            X_mediation_layer, 
            X_reference, 
            resolved_force);
      };

      X_ml = rk4.ForwardIntegrate(rk4_fun, X_ml, ts);

      // proposed_state->Read(key, proposed_trajectory);
      // updated_state->Write(key, trajectory);
      
      updated_trajectory_hist.emplace_back(
          PVAY2D(
            Vec2D(X_ml(0), X_ml(1)),
            Vec2D(X_ml(2), X_ml(3)),
            Vec2D(X_ml(4), X_ml(5)),
            0),
          proposed_trajectory.data_[idx].time_);

      if(++idx == proposed_trajectory.data_.size()) { break; }
    }

    timer.Stop();
    const Trajectory2D updated_trajectory(updated_trajectory_hist);
    Trajectory2DView view(updated_trajectory, potential_views);
    view.DisplayDynamics();
    view.DisplayPlots();

    return true;
  }

  bool MediationLayer2D::Stop() {
    this->ok_ = false;
    return true;
  }
}
