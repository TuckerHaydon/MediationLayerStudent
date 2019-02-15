// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <vector>

namespace mediation_layer {
  class Potential2DView {
    public:
      virtual ~Potential2DView() = default;
      virtual std::vector<visualization_msgs::Marker> Markers() const = 0;
  };
}
