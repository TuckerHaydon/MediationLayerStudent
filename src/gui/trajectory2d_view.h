// Author: Tucker Haydon

#ifndef PATH_PLANNING_GUI_TRAJECTORY2D_VIEW_H
#define PATH_PLANNING_GUI_TRAJECTORY2D_VIEW_H

#include "gnuplot-iostream.h"
#include "trajectory2d.h"
#include "potential2d_view.h"

#include <thread>
#include <chrono>

namespace path_planning {
  namespace {
    void DisplayPlots(const Trajectory2D& trajectory) {
      std::vector<boost::tuple<double, double, double, double, double, double, double>> hist;

      for(const TimeStampedPVAY2D& tspvay2D: trajectory.data_) {
        hist.emplace_back(
            tspvay2D.time_, 
            tspvay2D.pvay_.position_.x(), 
            tspvay2D.pvay_.position_.y(),
            tspvay2D.pvay_.velocity_.x(),
            tspvay2D.pvay_.velocity_.y(),
            tspvay2D.pvay_.acceleration_.x(),
            tspvay2D.pvay_.acceleration_.y()
            );
      }

      Gnuplot gp;
      gp << "set multiplot layout 3, 2" << std::endl;

      gp << "set title 'x position'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'time (s)'" << std::endl;
      gp << "plot " << gp.file1d(hist) << " using 1:2 with lines" << std::endl;

      gp << "set title 'y position'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'time (s)'" << std::endl;
      gp << "plot " << gp.file1d(hist) << " using 1:3 with lines" << std::endl;

      gp << "set title 'x velocity'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'time (s)'" << std::endl;
      gp << "plot " << gp.file1d(hist) << " using 1:4 with lines" << std::endl;

      gp << "set title 'y velocity'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'time (s)'" << std::endl;
      gp << "plot " << gp.file1d(hist) << " using 1:5 with lines" << std::endl;

      gp << "set title 'x acceleration'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'time (s)'" << std::endl;
      gp << "plot " << gp.file1d(hist) << " using 1:6 with lines" << std::endl;

      gp << "set title 'y acceleration'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'time (s)'" << std::endl;
      gp << "plot " << gp.file1d(hist) << " using 1:7 with lines" << std::endl;
    }

    void DisplayDynamics(
        const Trajectory2D& trajectory,
        const std::vector<std::shared_ptr<Potential2DView>> potential_views) {

      std::vector<boost::tuple<double, double, double, double, double, double, double>> hist;
      for(const TimeStampedPVAY2D& tspvay2D: trajectory.data_) {
        hist.emplace_back(
            tspvay2D.time_, 
            tspvay2D.pvay_.position_.x(), 
            tspvay2D.pvay_.position_.y(),
            tspvay2D.pvay_.velocity_.x(),
            tspvay2D.pvay_.velocity_.y(),
            tspvay2D.pvay_.acceleration_.x(),
            tspvay2D.pvay_.acceleration_.y()
            );
      }

      double time = hist.front().get<0>();

      Gnuplot gp;
      gp << "set title '2D Trajectory'" << std::endl;
      gp << "unset key" << std::endl;
      gp << "set xlabel 'x (m)'" << std::endl;
      gp << "set ylabel 'y (m)'" << std::endl;
      gp << "set xrange [-5:5]" << std::endl;
      gp << "set yrange [-5:5]" << std::endl;

      for(const auto& tup: hist) {
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * (tup.get<0>() - time))));
        time = tup.get<0>();

        std::vector<boost::tuple<double, double>> pos;
        pos.emplace_back(tup.get<1>(), tup.get<2>());

        gp << "set style line 3 linecolor rgb 'black' pt 7" << std::endl;
        gp << "plot " << gp.file1d(pos) << " using 1:2 with points linestyle 3, ";
        for(const auto& view: potential_views) {
          view->Display(gp);
        }
        gp << std::endl;
      }
    }
  }

  class Trajectory2DView {
    private:
      Trajectory2D trajectory_;
      std::vector<std::shared_ptr<Potential2DView>> potential_views_;

    public:
      Trajectory2DView(const Trajectory2D& trajectory, 
                       const std::vector<std::shared_ptr<Potential2DView>>& potential_views)
        : trajectory_(trajectory),
          potential_views_(potential_views) {}
      bool Display() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool Trajectory2DView::Display() const {
    DisplayPlots(this->trajectory_);
    DisplayDynamics(this->trajectory_, this->potential_views_);
    return true;
  }
}

#endif
