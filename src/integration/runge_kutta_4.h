// Author: Tucker Haydon

#ifndef PATH_PLANNING_INTEGRATION_RUNGE_KUTTA_4_H
#define PATH_PLANNING_INTEGRATION_RUNGE_KUTTA_4_H

#include <functional>

#include "time_span.h"

namespace path_planning {
  /*
   * 4th-order RungeKutta integrator
   * Reference: http://mathworld.wolfram.com/Runge-KuttaMethod.html
   */
  template<class T>
  class RungeKutta4 {
    private:
      T ForwardIntegrateStep(const std::function<T (double, const T&)>& f,
                             const T& y0,
                             const TimeSpan& ts) const { 
        const T k1 = ts.dt_ * f(ts.t0_         , y0);
        const T k2 = ts.dt_ * f(ts.t0_ + 0.5*ts.dt_, y0 + 0.5*k1);
        const T k3 = ts.dt_ * f(ts.t0_ + 0.5*ts.dt_, y0 + 0.5*k2);
        const T k4 = ts.dt_ * f(ts.t0_ + 1.0*ts.dt_, y0 + 1.0*k3);

        constexpr double c1 = (1.0/6.0), c2 = (1.0/3.0), c3 = (1.0/3.0), c4 = (1.0/6.0);

        return y0 + c1*k1 + c2*k2 + c3*k3 + c4*k4;
      };

    public:
      T ForwardIntegrate(const std::function<T (double, const T&)>& f, 
                         const T& y0, 
                         const TimeSpan& ts) const {
        // Integrate
        T retval = y0;
        double t = ts.t0_;
        for(; t <= ts.tf_ - ts.dt_; t += ts.dt_) {
          retval = ForwardIntegrateStep(f, retval, TimeSpan(t, t + ts.dt_));
        }

        // If did not integrate to end, take one last step
        if(t < ts.tf_) { 
          retval = ForwardIntegrateStep(f, retval, TimeSpan(t, ts.tf_ - t));
        }

        return retval;
      }
  };
}

#endif
