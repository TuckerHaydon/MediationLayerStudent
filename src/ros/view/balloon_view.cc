// Author: Tucker Haydon

#include "balloon_view.h"

namespace game_engine {
  std::vector<visualization_msgs::Marker> BalloonView::Markers() const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->options_.frame_id;
    marker.id = this->unique_id_;
    marker.ns = "Balloon";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 15.0f;
    marker.scale.y = 15.0f;
    marker.scale.z = 15.0f;
    marker.pose.position.x = this->balloon_position_.x();
    marker.pose.position.y= this->balloon_position_.y();
    marker.pose.position.z = this->balloon_position_.z() - 0.3; // Heuristic offset
    marker.color.r = this->options_.r;
    marker.color.g = this->options_.g;
    marker.color.b = this->options_.b;
    marker.color.a = this->options_.a;
    marker.mesh_resource = this->options_.mesh_resource;
    marker.mesh_use_embedded_materials = true;
     
    return {marker};
  }

  uint32_t BalloonView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
