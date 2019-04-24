// Author: Tucker Haydon

#include "line3d.h"

namespace mediation_layer {
  const Point3D& Line3D::Start() const {
    return this->start_;
  }

  const Point3D& Line3D::End() const {
    return this->end_;
  }

  Point3D Line3D::AsVector() const {
    return this->end_ - this->start_;
  }
}
