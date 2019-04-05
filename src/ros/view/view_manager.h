// Author: Tucker Haydon

#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <vector>

#include "visualization_msgs/"

#include "plane3d_view.h"
#include "polyhedron_view.h"

#include "marker_publisher_node.h"

namespace mediation_layer {
  // The ViewManager is a convenience object that encapsulates all of the code
  // used to publish views to RViz. 
  //
  // The ViewManager should run as its own thread.
  template<size_t T>
  class ViewManager {
    public:
      struct Options {
        Plane3DView::Options ground_view_options;
        Plane3DView::Options wall_view_options;
        PolyhedronView::Options obstacle_view_options;

        Options() {}
      };

      ViewManager(
          const Options& options = Options())
        : options_(options){}

      void Run();
      void Stop();

    private:
      void RunEnvironmentPublisher();
      void RunPotentialPublisher();
      void RunQuadPublisher();

      std::atomic<bool> ok_{true;}
      Options options_;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  void ViewManager<T>::Run() {
    std::thread environment_publisher_thread(
        [&]() {
          RunEnvironmentPublisher();
        });

    std::thread potential_publisher_thread(
        [&]() {
          RunPotentialPublisher();
        });

    std::thread quad_publisher_thread(
        [&]() {
          RunQuadPublisher();
        });

    environment_publisher_thread.join();
    potential_publisher_thread.join();
    quad_publisher_thread.join();
  }

  template <size_t T>
  void ViewManager<T>::RunEnvironmentPublisher() {
    // Setup
    std::vector<Plane3DView> plane_views;
    for(const Plane3D& wall: map.Walls()) {  
      plane_views.emplace_back(wall, wall_view_options);
    }
    plane_views.emplace_back(map.Ground(), ground_view_options);

    std::vector<PolyhedronView> obstacle_views;
    for(const Polyhedron& obstacle: map.Obstacles()) {
      obstacle_views.emplace_back(obstacle, obstacle_view_options);
    }

    auto environment_publisher = std::make_shared<MarkerPublisherNode>("environment");

    // Main loop
    while(this->ok_) {
      for(const visualization_msgs::Marker& marker: ground_view.Markers()) {
        environment_publisher->Publish(marker);
      }

      for(const Plane3DView& wall_view: wall_views) {
        for(const visualization_msgs::Marker& marker: wall_view.Markers()) {
          environment_publisher->Publish(marker);
        }
      }

      for(const PolyhedronView& obstacle_view: obstacle_views) {
        for(const visualization_msgs::Marker& marker: obstacle_view.Markers()) {
          environment_publisher->Publish(marker);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  void ViewManager<T>::RunPotentialPublisher() {

  }


  template <size_t T>
  void ViewManager<T>::Stop() {
    this->ok_ = false;
  }

  using ViewManager2D = ViewManager<2>;
  using ViewManager3D = ViewManager<3>;
}
