// Author: Tucker Haydon

#pragma once

#include <algorithm>

#include "line2d.h"
#include "potential2d.h"

namespace path_planning {
  class Line2DPotential : public Potential2D {
    public:
      struct Options {
        double activation_dist = 1.0;
        double min_dist = 0.5;
        double scale = 0.1;
        double activate_behind = false;
      };

      Line2DPotential(const Line2D& line, 
                      const Options& options)
        : line_(line),
          options_(options) {}

      Vec2D Resolve(const Point2D& point) const;

    private:
      Line2D line_;
      Options options_;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  Vec2D Line2DPotential::Resolve(const Point2D& point) const {
    // If the point if not within the orthogonal line bounds, no force
    if(false == this->line_.ProjectedContains(point))
    { return Point2D(0,0); }

    // If the point is on the right side of the line and the activation flag is
    // not set, no force
    if(false == this->options_.activate_behind &&
       false == this->line_.OnLeftSide(point))
    { return Point2D(0,0); }

    // Force direction is along the orthogonal unit vector
    const Vec2D unit = this->line_.OrthogonalUnitVector();
    const Vec2D orthogonal_point = this->line_.NormalIntersectionPoint(point);
    const double dist = (point - orthogonal_point).norm();

    return this->options_.scale * std::max
      (0.0, 
        (
         1.0/std::pow(dist - this->options_.min_dist, 2) - 
         1.0/std::pow(this->options_.activation_dist - this->options_.min_dist, 2))
        ) * 
      unit;
  } 
}
