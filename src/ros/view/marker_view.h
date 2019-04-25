// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <vector>

namespace game_engine {
  // MarkerView specifies interface functions for View objects that publish
  // markers
  class MarkerView {
    public:
      virtual ~MarkerView() = default;
      virtual std::vector<visualization_msgs::Marker> Markers() const = 0;
  };
}
