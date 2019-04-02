// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <mutex>

#include "marker_view.h"
#include "polyhedron.h"

namespace mediation_layer {
  // Ros view object for a polyhedron
  class PolyhedronView : public MarkerView {
    public:
      struct Options {
        // ROS frame
        std::string frame_id = "world";
        // RGB red value [0,1]
        float r = 0.0f;
        // RGB green value [0,1]
        float g = 0.0f;
        // RGB blue value [0,1]
        float b = 0.0f;
        // RGB alpha value [0,1]
        float a = 0.2f;

        Options() {}
      };

      PolyhedronView(
          const Options& options = Options(),
          const Polyhedron& polyhedron = Polyhedron())
        : options_(options),
          polyhedron_(polyhedron),
          unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

    private: 
      Polyhedron polyhedron_;
      Options options_;
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  std::vector<visualization_msgs::Marker> PolyhedronView::Markers() const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->options_.frame_id;
    marker.id = this->unique_id_;
    marker.ns = "Polyhedron";
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0f;
    marker.scale.y = 1.0f;
    marker.scale.z = 1.0f;
    marker.color.r = this->options_.r;
    marker.color.g = this->options_.g;
    marker.color.b = this->options_.b;
    marker.color.a = this->options_.a;

    // Iterate through all of the faces on the polyhedron and draw triangles
    // between the vertices and some point on the interior of the face. The
    // interior point is arbitrarily selected as the first vertex.
    for(const Plane3D& face: this->polyhedron_.Faces()) { 
      const Point3D interior_point = face.Edges()[0].Start();
      for(const Line3D& edge: face.Edges()) {
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
    }
     
    return {marker};
  }

  inline uint32_t PolyhedronView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
