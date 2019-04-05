// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <mutex>

#include "marker_view.h"
#include "quad_state_guard.h"

namespace mediation_layer {
  template <size_t T>
  class QuadView : public MarkerView {
    public:
      struct Options {
        // ROS frame
        std::string frame_id = "world";
        // Mesh resource file
        std::string mesh_resource = "";
        // RGB red value [0,1]
        float r = 1.0f;
        // RGB green value [0,1]
        float g = 1.0f;
        // RGB blue value [0,1]
        float b = 1.0f;
        // RGB alpha value [0,1]
        float a = 1.0f;

        Options() {}
      };

      QuadView(
          std::shared_ptr<QuadStateGuard<T>> quad_state_guard = nullptr,
          const Options& options = Options())
        : quad_state_guard_(quad_state_guard),
          options_(options),
          unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

    private: 
      Options options_;
      std::shared_ptr<QuadStateGuard<T>> quad_state_guard_;
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline std::vector<visualization_msgs::Marker> QuadView<T>::Markers() const {
    QuadState<T> quad_state;
    this->quad_state_guard_->Read(quad_state);

    const Eigen::Vector<double, T> quad_position = quad_state.Position();

    visualization_msgs::Marker marker;
    marker.header.frame_id = this->options_.frame_id;
    marker.id = this->unique_id_;
    marker.ns = "Quad";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0f;
    marker.scale.y = 1.0f;
    marker.scale.z = 1.0f;
    marker.pose.position.x = quad_position.x();
    marker.pose.position.y = quad_position.y();
    marker.pose.position.z = (T == 2 ? 0 : quad_position(2));
    marker.color.r = this->options_.r;
    marker.color.g = this->options_.g;
    marker.color.b = this->options_.b;
    marker.color.a = this->options_.a;
    marker.mesh_resource = this->options_.mesh_resource;
    marker.mesh_use_embedded_materials = true;
     
    return {marker};
  }

  template <size_t T>
  inline uint32_t QuadView<T>::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }

  using QuadView2D = QuadView<2>;
  using QuadView3D = QuadView<3>;
}
