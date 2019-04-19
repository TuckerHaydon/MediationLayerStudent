// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <mutex>

#include "marker_view.h"
#include "quad_state_guard.h"

namespace mediation_layer {
  class QuadView : public MarkerView {
    public:
      struct Options {
        // ROS frame
        std::string frame_id;
        // Mesh resource file
        std::string mesh_resource;
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
            const std::string& mesh_resource_ = "",
            const float r_ = 1.0f,
            const float g_ = 1.0f,
            const float b_ = 1.0f,
            const float a_ = 1.0f
            ) 
        : frame_id(frame_id_),
          mesh_resource(mesh_resource_),
          r(r_),
          g(g_),
          b(b_),
          a(a_)
        {}
      };

      QuadView(
          std::shared_ptr<QuadStateGuard> quad_state_guard = nullptr,
          const Options& options = Options())
        : quad_state_guard_(quad_state_guard),
          options_(options),
          unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

    private: 
      Options options_;
      std::shared_ptr<QuadStateGuard> quad_state_guard_;
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline std::vector<visualization_msgs::Marker> QuadView::Markers() const {
    QuadState quad_state;
    this->quad_state_guard_->Read(quad_state);

    const Eigen::Vector<double, 3> quad_position = quad_state.Position();

    visualization_msgs::Marker marker;
    marker.header.frame_id = this->options_.frame_id;
    marker.id = this->unique_id_;
    marker.ns = "Quad";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.6f;
    marker.scale.y = 0.6f;
    marker.scale.z = 0.6f;
    marker.pose.position.x = quad_position.x();
    marker.pose.position.y = quad_position.y();
    marker.pose.position.z = quad_position.z();
    marker.color.r = this->options_.r;
    marker.color.g = this->options_.g;
    marker.color.b = this->options_.b;
    marker.color.a = this->options_.a;
    marker.mesh_resource = this->options_.mesh_resource;
    marker.mesh_use_embedded_materials = true;
     
    return {marker};
  }

  inline uint32_t QuadView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
