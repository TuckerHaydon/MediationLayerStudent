// Author: Tucker Haydon

#pragma once

#include <algorithm>

#include "point2d.h"
#include "potential2d.h"
#include "vec2d.h"

namespace path_planning {

  class Point2DPotential : public Potential2D {
    public:
      struct Options {
        double activation_dist = 1.0;
        double min_dist = 0.5;
        double scale = 0.1;
      };

      Point2DPotential(const Point2D& point, 
                       const Options& options)
        : point_(point),
          options_(options) {}

      Vec2D Resolve(const Point2D& point) const;

    private:
      Point2D point_;
      Options options_;
      friend class Point2DPotentialView;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  Vec2D Point2DPotential::Resolve(const Point2D& point) const {
    const Vec2D unit = (point - this->point_).normalized();
    const double dist = (point - this->point_).norm();

    return this->options_.scale * std::max
      (0.0, 
        (
         1.0/std::pow(dist - this->options_.min_dist, 2) - 
         1.0/std::pow(this->options_.activation_dist - this->options_.min_dist, 2))
        ) * 
      unit;
  } 

}
