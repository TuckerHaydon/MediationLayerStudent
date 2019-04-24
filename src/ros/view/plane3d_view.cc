// Author: Tucker Haydon

#include "plane3d_view.h"

namespace mediation_layer {
  std::vector<visualization_msgs::Marker> Plane3DView::Markers() const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->options_.frame_id;
    marker.ns = "Plane3d";
    marker.id = this->unique_id_;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0f;
    marker.scale.y = 1.0f;
    marker.scale.z = 1.0f;
    marker.color.r = this->options_.r;
    marker.color.g = this->options_.g;
    marker.color.b = this->options_.b;
    marker.color.a = this->options_.a;

    const Point3D interior_point = this->plane_.Edges()[0].Start();
    for(const Line3D& edge: this->plane_.Edges()) {
      geometry_msgs::Point p1;
      p1.x = interior_point.x();
      p1.y = interior_point.y();
      p1.z = interior_point.z();

      geometry_msgs::Point p2;
      p2.x = edge.Start().x();
      p2.y = edge.Start().y();
      p2.z = edge.Start().z();

      geometry_msgs::Point p3;
      p3.x = edge.End().x();
      p3.y = edge.End().y();
      p3.z = edge.End().z();

      marker.points.push_back(p1);
      marker.points.push_back(p2);
      marker.points.push_back(p3);
    }
     
    return {marker};
  }

  uint32_t Plane3DView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}

