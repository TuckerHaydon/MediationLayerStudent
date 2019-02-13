// Author: Tucker Haydon

#ifndef PATH_PLANNING_GUI_TRAJECTORY2D_VIEW_H
#define PATH_PLANNING_GUI_TRAJECTORY2D_VIEW_H

#include "gnuplot-iostream.h"
#include "trajectory2d.h"
#include "potential2d_view.h"
// #include "marker_publisher_node.h"
// #include "polygon_publisher_node.h"

#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace path_planning {
  class Trajectory2DView {
    private:
      Trajectory2D trajectory_;
      std::vector<std::shared_ptr<Potential2DView>> potential_views_;

    public:
      Trajectory2DView(const Trajectory2D& trajectory, 
                       const std::vector<std::shared_ptr<Potential2DView>>& potential_views)
        : trajectory_(trajectory),
          potential_views_(potential_views) {}
      bool DisplayPlots() const;
      bool DisplayDynamics() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  bool Trajectory2DView::DisplayPlots() const {
    std::vector<boost::tuple<double, double, double, double, double, double, double>> hist;

    for(const TimeStampedPVAY2D& tspvay2D: this->trajectory_.data_) {
      hist.emplace_back(
          tspvay2D.time_, 
          tspvay2D.pvay_.position_.x(), 
          tspvay2D.pvay_.position_.y(),
          tspvay2D.pvay_.velocity_.x(),
          tspvay2D.pvay_.velocity_.y(),
          tspvay2D.pvay_.acceleration_.x(),
          tspvay2D.pvay_.acceleration_.y()
          );
    }

    Gnuplot gp;
    gp << "set multiplot layout 3, 2" << std::endl;

    gp << "set title 'x position'" << std::endl;
    gp << "unset key" << std::endl;
    gp << "set xlabel 'time (s)'" << std::endl;
    gp << "plot " << gp.file1d(hist) << " using 1:2 with lines" << std::endl;

    gp << "set title 'y position'" << std::endl;
    gp << "unset key" << std::endl;
    gp << "set xlabel 'time (s)'" << std::endl;
    gp << "plot " << gp.file1d(hist) << " using 1:3 with lines" << std::endl;

    gp << "set title 'x velocity'" << std::endl;
    gp << "unset key" << std::endl;
    gp << "set xlabel 'time (s)'" << std::endl;
    gp << "plot " << gp.file1d(hist) << " using 1:4 with lines" << std::endl;

    gp << "set title 'y velocity'" << std::endl;
    gp << "unset key" << std::endl;
    gp << "set xlabel 'time (s)'" << std::endl;
    gp << "plot " << gp.file1d(hist) << " using 1:5 with lines" << std::endl;

    gp << "set title 'x acceleration'" << std::endl;
    gp << "unset key" << std::endl;
    gp << "set xlabel 'time (s)'" << std::endl;
    gp << "plot " << gp.file1d(hist) << " using 1:6 with lines" << std::endl;

    gp << "set title 'y acceleration'" << std::endl;
    gp << "unset key" << std::endl;
    gp << "set xlabel 'time (s)'" << std::endl;
    gp << "plot " << gp.file1d(hist) << " using 1:7 with lines" << std::endl;

    return true;
  }

  bool Trajectory2DView::DisplayDynamics() const {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "TempNode", ros::init_options::NoSigintHandler);
    // MarkerPublisherNode marker_pub;
    ros::NodeHandle nh("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 100);



    // Visualize the trajectory
    std::vector<boost::tuple<double, double, double, double, double, double, double>> hist;
    for(const TimeStampedPVAY2D& tspvay2D: this->trajectory_.data_) {
      hist.emplace_back(
          tspvay2D.time_, 
          tspvay2D.pvay_.position_.x(), 
          tspvay2D.pvay_.position_.y(),
          tspvay2D.pvay_.velocity_.x(),
          tspvay2D.pvay_.velocity_.y(),
          tspvay2D.pvay_.acceleration_.x(),
          tspvay2D.pvay_.acceleration_.y()
          );
    }

    double time = hist.front().get<0>();

    for(const auto& tup: hist) {
      for(const std::shared_ptr<Potential2DView>& view: this->potential_views_) {
        const std::vector<visualization_msgs::Marker> markers = view->Markers();
        for(const visualization_msgs::Marker& marker: markers) {
          marker_pub.publish(marker);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * (tup.get<0>() - time))));
      time = tup.get<0>();

      visualization_msgs::Marker marker;
        
      marker.header.frame_id = "/world";
      marker.header.stamp = ros::Time::now();
      
      marker.ns = "basic_shapes";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = tup.get<1>();
      marker.pose.position.y = tup.get<2>();
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.01;
      
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();
      marker_pub.publish(marker);
    }

    return true;
  }
}

#endif
