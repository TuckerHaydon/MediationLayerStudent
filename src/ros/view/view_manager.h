// Author: Tucker Haydon

#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <vector>

#include "marker_publisher_node.h"
#include "polyhedron_view.h"
#include "plane3d_view.h"

#include "plane3d_potential.h"
#include "plane3d_potential_view.h"

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
        Plane3DPotentialView::Options plane3d_potential_view_options{
            "world", 1.0f, 0.0f, 0.0f, 0.1f};

        std::vector<std::shared_ptr<Potential>> potentials;
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

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline void ViewManager::Run(
      const QuadViewOptions quad_view_options,
      const BalloonViewOptions balloon_view_options,
      const EnvironmentViewOptions environment_view_options) {

    std::thread quad_publisher_thread(
        [&]() {
          RunQuadPublisher(quad_view_options);
        });

    std::thread balloon_publisher_thread(
        [&]() {
          RunBalloonPublisher(balloon_view_options);
        });

    std::thread environment_publisher_thread(
        [&]() {
          RunEnvironmentPublisher(environment_view_options);
        });

    quad_publisher_thread.join();
    balloon_publisher_thread.join();
    environment_publisher_thread.join();
  }

  inline void ViewManager::RunQuadPublisher(
      const QuadViewOptions quad_view_options) {

    // Setup
    std::vector<QuadView> quad_views;

    for(const auto p: quad_view_options.quads) {
      if(p.first == "red") {
        QuadView::Options view_options;
        view_options.mesh_resource = quad_view_options.quad_mesh_file_path;
        view_options.r = 1.0f;
        view_options.g = 0.0f;
        view_options.b = 0.0f;
        quad_views.emplace_back(p.second, view_options);
      }
      else if(p.first == "blue") {
        QuadView::Options view_options;
        view_options.mesh_resource = quad_view_options.quad_mesh_file_path;
        view_options.r = 0.0f;
        view_options.g = 0.0f;
        view_options.b = 1.0f;
        quad_views.emplace_back(p.second, view_options);
      }
    }

    auto quads_publisher = std::make_shared<MarkerPublisherNode>("quads");

    // Main loop
    // 50 Hz. The quads move quickly and update often
    while(this->ok_) {
      for(const auto& view: quad_views) {
        for(const visualization_msgs::Marker& marker: view.Markers()) {
          quads_publisher->Publish(marker);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  inline void ViewManager::RunBalloonPublisher(
      const BalloonViewOptions balloon_view_options) {

    // Setup
    std::vector<BalloonView> balloon_views;

    for(const auto p: balloon_view_options.balloons) {
      if(p.first == "red") {
        BalloonView::Options view_options;
        view_options.mesh_resource = balloon_view_options.balloon_mesh_file_path;
        view_options.r = 1.0f;
        view_options.g = 0.0f;
        view_options.b = 0.0f;
        balloon_views.emplace_back(p.second, view_options);
      }
      else if(p.first == "blue") {
        BalloonView::Options view_options;
        view_options.mesh_resource = balloon_view_options.balloon_mesh_file_path;
        view_options.r = 0.0f;
        view_options.g = 0.0f;
        view_options.b = 1.0f;
        balloon_views.emplace_back(p.second, view_options);
      }
    }

    auto balloons_publisher = std::make_shared<MarkerPublisherNode>("balloons");

    // Main loop
    // 50 Hz. 
    while(this->ok_) {
      for(const auto& view: balloon_views) {
        for(const visualization_msgs::Marker& marker: view.Markers()) {
          balloons_publisher->Publish(marker);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  inline void ViewManager::RunEnvironmentPublisher(
      const EnvironmentViewOptions environment_view_options) {

    // Setup
    std::vector<Plane3DView> plane_views;
    for(const Plane3D& wall: environment_view_options.map.Walls()) {  
      plane_views.emplace_back(
          wall, 
          environment_view_options.wall_view_options);
    }
    plane_views.emplace_back(
        environment_view_options.map.Ground(), 
        environment_view_options.ground_view_options);

    std::vector<PolyhedronView> obstacle_views;
    std::vector<Plane3DPotentialView> plane_potential_views;
    for(const Polyhedron& obstacle: environment_view_options.map.Obstacles()) {
      obstacle_views.emplace_back(
          obstacle, 
          environment_view_options.obstacle_view_options);
    }

    for(std::shared_ptr<Potential> potential: environment_view_options.potentials) {
      plane_potential_views.emplace_back(
          std::dynamic_pointer_cast<Plane3DPotential>(potential), 
          environment_view_options.plane3d_potential_view_options);
    }

    auto environment_publisher = std::make_shared<MarkerPublisherNode>("environment");
    auto potentials_publisher = std::make_shared<MarkerPublisherNode>("potentials");

    // Main loop
    // 2 Hz. Environment does not change often
    while(this->ok_) {
      for(const Plane3DView& view: plane_views) {
        for(const visualization_msgs::Marker& marker: view.Markers()) {
          environment_publisher->Publish(marker);
        }
      }

      for(const PolyhedronView& obstacle_view: obstacle_views) {
        for(const visualization_msgs::Marker& marker: obstacle_view.Markers()) {
          environment_publisher->Publish(marker);
        }
      }

      for(const auto& view: plane_potential_views) {
        for(const visualization_msgs::Marker& marker: view.Markers()) {
          potentials_publisher->Publish(marker);
        }
      } 
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }


  void ViewManager::Stop() {
    this->ok_ = false;
  }
}
