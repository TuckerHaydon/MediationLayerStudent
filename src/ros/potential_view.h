// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <vector>

namespace mediation_layer {
  class PotentialView {
    public:
      virtual ~PotentialView() = default;
      virtual std::vector<visualization_msgs::Marker> Markers() const = 0;
  };
}
