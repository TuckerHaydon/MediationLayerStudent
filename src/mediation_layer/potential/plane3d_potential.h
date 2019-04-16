// Author: Tucker Haydon

#pragma once

#include <memory>
#include <vector>

#include "potential.h"
#include "plane3d.h"
#include "point3d_potential.h"

namespace mediation_layer {
  class Plane3DPotential : public Potential {
    public:
      struct Options {
        double activation_dist = 1.0;
        double min_dist = 0.5;
        double scale = 0.1;

        Options() {}
      };

      Plane3DPotential(const Plane3D& plane = Plane3D(),
                       const Options& options = Options())
        : plane_(plane),
          options_(options) {
      }

      Vec3D Resolve(const Point3D& point) const override;

    private:
      Plane3D plane_;
      std::vector<std::shared_ptr<Point3DPotential>> vertex_potentials_;
      Options options_;
      friend class Plane3DPotentialView;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Vec3D Plane3DPotential::Resolve(const Point3D& point) const {
    const Point3D closest_point = this->plane_.ClosestPoint(point);
    const Vec3D unit = (point - closest_point).normalized();
    const double dist = (point - closest_point).norm();

    // If the vector is not in the plane's 'wake',the area orthogonal to the
    // plane and contained within the plane's convex area, the plane exerts no
    // force
    if(false == this->plane_.Contains(closest_point)) {
      return Vec3D(0,0,0);
    }

    return this->options_.scale * std::max
      (0.0, 
        (
         1.0/std::pow(dist - this->options_.min_dist, 2) - 
         1.0/std::pow(this->options_.activation_dist - this->options_.min_dist, 2))
        ) * 
      unit;
  } 

}
