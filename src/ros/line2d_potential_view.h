// Author: Tucker Haydon

#pragma once

#include <memory>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "point2d_potential.h"

namespace mediation_layer {
  class Line2DPotentialView : public PotentialView {
    private:
      static uint32_t GenerateUniqueId();

      std::shared_ptr<Line2DPotential> potential_;
      uint32_t id_min_, id_max_;

    public:
      Line2DPotentialView(const std::shared_ptr<Line2DPotential>& potential)
        : potential_(potential),
          id_min_(Line2DPotentialView::GenerateUniqueId()),
          id_max_(Line2DPotentialView::GenerateUniqueId()) {}
      std::vector<visualization_msgs::Marker> Markers() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline std::vector<visualization_msgs::Marker> Line2DPotentialView::Markers() const {
    visualization_msgs::Marker max_rectangle, min_rectangle;
    const double max_length = this->potential_->options_.activation_dist;
    const double min_length = this->potential_->options_.min_dist;

    const Line2D& line = this->potential_->line_;

    const Point2D max_center = line.Midpoint() + max_length/2 * line.OrthogonalUnitVector();
    const double max_scale_x = 2*(max_center - line.Start()).x();
    const double max_scale_y = 2*(max_center - line.Start()).y();

    const Point2D min_center = line.Midpoint() + min_length/2 * line.OrthogonalUnitVector();
    const double min_scale_x = 2*(min_center - line.Start()).x();
    const double min_scale_y = 2*(min_center - line.Start()).y();

    max_rectangle.header.frame_id = "/world";
    max_rectangle.header.stamp = ros::Time::now();
    max_rectangle.ns = "Line2DPotentialView";
    max_rectangle.id = this->id_max_;
    max_rectangle.type = visualization_msgs::Marker::CUBE;
    max_rectangle.action = visualization_msgs::Marker::ADD;
    max_rectangle.pose.position.x = max_center.x();
    max_rectangle.pose.position.y = max_center.y();
    max_rectangle.pose.position.z = 0;
    max_rectangle.pose.orientation.x = 0.0;
    max_rectangle.pose.orientation.y = 0.0;
    max_rectangle.pose.orientation.z = 0.0;
    max_rectangle.pose.orientation.w = 1.0;
    max_rectangle.scale.x = max_scale_x;
    max_rectangle.scale.y = max_scale_y;
    max_rectangle.scale.z = 0.01;
    max_rectangle.color.r = 1.0f;
    max_rectangle.color.g = 0.0f;
    max_rectangle.color.b = 0.0f;
    max_rectangle.color.a = 0.5f;
    max_rectangle.lifetime = ros::Duration();

    min_rectangle.header.frame_id = "/world";
    min_rectangle.header.stamp = ros::Time::now();
    min_rectangle.ns = "Line2DPotentialView";
    min_rectangle.id = this->id_min_;
    min_rectangle.type = visualization_msgs::Marker::CUBE;
    min_rectangle.action = visualization_msgs::Marker::ADD;
    min_rectangle.pose.position.x = min_center.x();
    min_rectangle.pose.position.y = min_center.y();
    min_rectangle.pose.position.z = 0;
    min_rectangle.pose.orientation.x = 0.0;
    min_rectangle.pose.orientation.y = 0.0;
    min_rectangle.pose.orientation.z = 0.0;
    min_rectangle.pose.orientation.w = 1.0;
    min_rectangle.scale.x = min_scale_x;
    min_rectangle.scale.y = min_scale_y;
    min_rectangle.scale.z = 0.02;
    min_rectangle.color.r = 0.0f;
    min_rectangle.color.g = 0.0f;
    min_rectangle.color.b = 0.0f;
    min_rectangle.color.a = 1.0f;
    min_rectangle.lifetime = ros::Duration();

    return {max_rectangle, min_rectangle};
  }

  inline uint32_t Line2DPotentialView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
