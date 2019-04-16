// Author: Tucker Haydon

#pragma once

#include <algorithm>

#include "potential.h"

namespace mediation_layer {

  class Point3DPotential : public Potential {
    public:
      struct Options {
        double activation_dist = 1.0;
        double min_dist = 0.5;
        double scale = 0.1;
      };

      Point3DPotential(const Point3D& point, 
                       const Options& options)
        : point_(point),
          options_(options) {}

      Vec3D Resolve(const Point3D& point) const override;

    private:
      Point3D point_;
      Options options_;
      friend class Point3DPotentialView;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Vec3D Point3DPotential::Resolve(const Point3D& point) const {
    const Vec3D unit = (point - this->point_).normalized();
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
