// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <memory>

#include "marker_view.h"
#include "plane3d_potential.h"

namespace mediation_layer {
  class Plane3DPotentialView : public MarkerView {
    public:
      struct Options {
        // ROS frame
        std::string frame_id = "world";
        // RGB red value [0,1]
        float r = 1.0f;
        // RGB green value [0,1]
        float g = 0.0f;
        // RGB blue value [0,1]
        float b = 0.0f;
        // RGB alpha value [0,1]
        float a = 0.2f;

        Options() {}
      };

      Plane3DPotentialView(
          const std::shared_ptr<Plane3DPotential> potential = nullptr,
          const Options& options = Options())
        : potential_(potential),
          options_(options),
          unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

    private: 
      std::shared_ptr<Plane3DPotential> potential_;
      Options options_;
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  std::vector<visualization_msgs::Marker> Plane3DPotentialView::Markers() const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->options_.frame_id;
    marker.ns = "Plane3dPotential";
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

    // Game Plan: Fill triangles for the base. Create a copy plane at a set distance
    //            along the normal. Fill another base. Fill triangles between
    //            the two
    const double offset_dist = this->potential_->options_.activation_dist;
    const Vec3D offset_vec = offset_dist * this->potential_->plane_.NormalVector();

    { // Fill base
      const Point3D interior_point = this->potential_->plane_.Edges()[0].Start();
      for(const Line3D& edge: this->potential_->plane_.Edges()) {
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

    { // Fill top
      const Point3D interior_point 
        = this->potential_->plane_.Edges()[0].Start() + offset_vec;
      for(const Line3D& edge: this->potential_->plane_.Edges()) {
        geometry_msgs::Point p1;
        p1.x = interior_point.x();
        p1.y = interior_point.y();
        p1.z = interior_point.z();

        geometry_msgs::Point p2;
        p2.x = edge.Start().x() + offset_vec.x();
        p2.y = edge.Start().y() + offset_vec.y();
        p2.z = edge.Start().z() + offset_vec.z();

        geometry_msgs::Point p3;
        p3.x = edge.End().x() + offset_vec.x();
        p3.y = edge.End().y() + offset_vec.y();
        p3.z = edge.End().z() + offset_vec.z();

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
      }
    }

    { // Fill sides
      for(const Line3D& edge: this->potential_->plane_.Edges()) {
        { // 1st triangle
          geometry_msgs::Point p1;
          p1.x = edge.Start().x();
          p1.y = edge.Start().y();
          p1.z = edge.Start().z();

          geometry_msgs::Point p2;
          p2.x = edge.End().x();
          p2.y = edge.End().y();
          p2.z = edge.End().z();

          geometry_msgs::Point p3;
          p3.x = edge.End().x() + offset_vec.x();
          p3.y = edge.End().y() + offset_vec.y();
          p3.z = edge.End().z() + offset_vec.z();

          marker.points.push_back(p1);
          marker.points.push_back(p2);
          marker.points.push_back(p3);
        }
        
        { // 2nd triangle
          geometry_msgs::Point p1;
          p1.x = edge.Start().x() + offset_vec.x();
          p1.y = edge.Start().y() + offset_vec.y();
          p1.z = edge.Start().z() + offset_vec.z();

          geometry_msgs::Point p2;
          p2.x = edge.Start().x();
          p2.y = edge.Start().y();
          p2.z = edge.Start().z();

          geometry_msgs::Point p3;
          p3.x = edge.End().x() + offset_vec.x();
          p3.y = edge.End().y() + offset_vec.y();
          p3.z = edge.End().z() + offset_vec.z();

          marker.points.push_back(p1);
          marker.points.push_back(p2);
          marker.points.push_back(p3);
        }
      }
    }
     
    return {marker};
  }

  inline uint32_t Plane3DPotentialView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
