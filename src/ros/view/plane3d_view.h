// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <mutex>

#include "marker_view.h"
#include "plane3d.h"

namespace game_engine {
  class Plane3DView : public MarkerView {
    public:
      struct Options {
        // ROS frame
        std::string frame_id;
        // RGB red value [0,1]
        float r;
        // RGB green value [0,1]
        float g;
        // RGB blue value [0,1]
        float b;
        // RGB alpha value [0,1]
        float a;

        Options(
            const std::string& frame_id_ = "world",
            const float r_ = 0.0f,
            const float g_ = 0.0f,
            const float b_ = 0.0f,
            const float a_ = 0.2f
            ) 
        : frame_id(frame_id_),
          r(r_),
          g(g_),
          b(b_),
          a(a_)
        {}
      };

      Plane3DView(
          const Plane3D& plane = Plane3D(),
          const Options& options = Options())
        : plane_(plane),
          options_(options),
          unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

    private: 
      Plane3D plane_;
      Options options_;
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };
}
