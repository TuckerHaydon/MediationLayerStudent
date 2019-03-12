// Author: Tucker Haydon

#pragma once

#include <memory>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "point2d_potential.h"

namespace mediation_layer {
  class Point2DPotentialView : public PotentialView {
    private:
      static uint32_t GenerateUniqueId();

      std::shared_ptr<Point2DPotential> potential_;
      uint32_t id_min_, id_max_;

    public:
      Point2DPotentialView(const std::shared_ptr<Point2DPotential>& potential)
        : potential_(potential),
          id_min_(Point2DPotentialView::GenerateUniqueId()),
          id_max_(Point2DPotentialView::GenerateUniqueId()) {}
      std::vector<visualization_msgs::Marker> Markers() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline std::vector<visualization_msgs::Marker> Point2DPotentialView::Markers() const {
    visualization_msgs::Marker max_circle, min_circle;
    const double max_radius = this->potential_->options_.activation_dist;
    const double min_radius = this->potential_->options_.min_dist;

    max_circle.header.frame_id = "/world";
    max_circle.header.stamp = ros::Time::now();
    max_circle.ns = "Point2DPotentialView";
    max_circle.id = this->id_max_;
    max_circle.type = visualization_msgs::Marker::CYLINDER;
    max_circle.action = visualization_msgs::Marker::ADD;
    max_circle.pose.position.x = this->potential_->point_.x();
    max_circle.pose.position.y = this->potential_->point_.y();
    max_circle.pose.position.z = 0;
    max_circle.pose.orientation.x = 0.0;
    max_circle.pose.orientation.y = 0.0;
    max_circle.pose.orientation.z = 0.0;
    max_circle.pose.orientation.w = 1.0;
    max_circle.scale.x = 2*max_radius;
    max_circle.scale.y = 2*max_radius;
    max_circle.scale.z = 0.01;
    max_circle.color.r = 1.0f;
    max_circle.color.g = 0.0f;
    max_circle.color.b = 0.0f;
    max_circle.color.a = 0.5f;
    max_circle.lifetime = ros::Duration();

    min_circle.header.frame_id = "/world";
    min_circle.header.stamp = ros::Time::now();
    min_circle.ns = "Point2DPotentialView";
    min_circle.id = this->id_min_;
    min_circle.type = visualization_msgs::Marker::CYLINDER;
    min_circle.action = visualization_msgs::Marker::ADD;
    min_circle.pose.position.x = this->potential_->point_.x();
    min_circle.pose.position.y = this->potential_->point_.y();
    min_circle.pose.position.z = 0;
    min_circle.pose.orientation.x = 0.0;
    min_circle.pose.orientation.y = 0.0;
    min_circle.pose.orientation.z = 0.0;
    min_circle.pose.orientation.w = 1.0;
    min_circle.scale.x = 2*min_radius;
    min_circle.scale.y = 2*min_radius;
    min_circle.scale.z = 0.02;
    min_circle.color.r = 0.0f;
    min_circle.color.g = 0.0f;
    min_circle.color.b = 0.0f;
    min_circle.color.a = 1.0f;
    min_circle.lifetime = ros::Duration();


    return {max_circle, min_circle};
  }

  inline uint32_t Point2DPotentialView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
