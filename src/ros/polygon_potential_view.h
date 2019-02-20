// Author: Tucker Haydon

#pragma once

#include <memory>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "polygon_potential.h"
#include "line2d_potential_view.h"

namespace mediation_layer {
  class PolygonPotentialView : public PotentialView {
    private:
      static uint32_t GenerateUniqueId();

      std::shared_ptr<PolygonPotential> potential_;
      uint32_t id_min_, id_max_;

      std::vector<Line2DPotentialView> edge_potential_views_;
      std::vector<Point2DPotentialView> vertex_potential_views_;

    public:
      PolygonPotentialView(const std::shared_ptr<PolygonPotential>& potential)
        : potential_(potential),
          id_min_(PolygonPotentialView::GenerateUniqueId()),
          id_max_(PolygonPotentialView::GenerateUniqueId()) {
            for(const std::shared_ptr<Line2DPotential>& edge_potential: this->potential_->edge_potentials_) {
              edge_potential_views_.emplace_back(edge_potential);
            }
            for(const std::shared_ptr<Point2DPotential>& vertex_potential: this->potential_->vertex_potentials_) {
              vertex_potential_views_.emplace_back(vertex_potential);
            }
          }
      std::vector<visualization_msgs::Marker> Markers() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline std::vector<visualization_msgs::Marker> PolygonPotentialView::Markers() const {
    std::vector<visualization_msgs::Marker> markers;
    for(const Line2DPotentialView& view: this->edge_potential_views_) {
      const std::vector<visualization_msgs::Marker> edge_markers = view.Markers();
      markers.insert(
          markers.end(),
          edge_markers.begin(),
          edge_markers.end()
          );
    }
    for(const Point2DPotentialView& view: this->vertex_potential_views_) {
      const std::vector<visualization_msgs::Marker> vertex_markers = view.Markers();
      markers.insert(
          markers.end(),
          vertex_markers.begin(),
          vertex_markers.end()
          );
    }
    return markers;
  }

  inline uint32_t PolygonPotentialView::GenerateUniqueId() {
    static std::mutex mtx;
    static uint32_t id = 0;

    std::lock_guard<std::mutex> lock(mtx);
    id++;
    return id;
  }
}
