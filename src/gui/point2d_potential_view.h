// Author: Tucker Haydon

#pragma once

#include <memory>

#include "point2d_potential.h"
#include "gnuplot-iostream.h"

namespace path_planning {
  class Point2DPotentialView : public Potential2DView {
    private:
      std::shared_ptr<Point2DPotential> potential_;

    public:
      Point2DPotentialView(const std::shared_ptr<Point2DPotential>& potential)
        : potential_(potential) {}
      Gnuplot& Display(Gnuplot& gp) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Gnuplot& Point2DPotentialView::Display(Gnuplot& gp) const {
    {
      std::vector<boost::tuple<double, double, double>> circles;
      circles.emplace_back(
          this->potential_->point_.x(),
          this->potential_->point_.y(),
          (this->potential_->options_.activation_dist - this->potential_->options_.min_dist)/2);

      gp << gp.file1d(circles) << " with circles lc rgb 'red' fs transparent solid 0.15 noborder, ";
    }
    {
      std::vector<boost::tuple<double, double, double>> circles;
      circles.emplace_back(
          this->potential_->point_.x(),
          this->potential_->point_.y(),
          this->potential_->options_.min_dist);

      gp << gp.file1d(circles) << " with circles lc rgb 'black' fs transparent solid 1 noborder, ";
    }
    return gp;
  }
}
