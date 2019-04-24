// Author: Tucker Haydon

#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <vector>

#include "map3d.h"

#include "marker_publisher_node.h"
#include "polyhedron_view.h"
#include "plane3d_view.h"

#include "quad_view.h"
#include "balloon_view.h"

namespace mediation_layer {
  // The ViewManager is a convenience object that encapsulates all of the code
  // used to publish views to RViz. 
  //
  // The ViewManager should run as its own thread.
  class ViewManager {
    public:
      struct BalloonViewOptions {
        std::string balloon_mesh_file_path;
        std::vector<std::pair<std::string, Eigen::Vector3d>> balloons;

        BalloonViewOptions() {}
      }; 

      struct QuadViewOptions {
        std::string quad_mesh_file_path;
        std::vector<std::pair<std::string, std::shared_ptr<QuadStateGuard>>> quads;

        QuadViewOptions() {}
      };

      struct EnvironmentViewOptions { 
        Plane3DView::Options ground_view_options{
          "world", 0.0f, 1.0f, 0.0f, 1.0f};
        Plane3DView::Options wall_view_options{
            "world", 0.0f, 0.0f, 0.0f, 0.1f};
        PolyhedronView::Options obstacle_view_options{
            "world", 1.0f, 1.0f, 1.0f, 1.0f};
        Map3D map;

        EnvironmentViewOptions() {}
      };

      ViewManager() {};

      void Run(
          const QuadViewOptions quad_view_options,
          const BalloonViewOptions balloon_view_options,
          const EnvironmentViewOptions environment_view_options);

      void Stop();

    private:
      void RunQuadPublisher(
          const QuadViewOptions quad_view_options);
      void RunBalloonPublisher(
          const BalloonViewOptions balloon_view_options);
      void RunEnvironmentPublisher(
          const EnvironmentViewOptions environment_view_options);

      volatile std::atomic<bool> ok_{true};
  };
}
