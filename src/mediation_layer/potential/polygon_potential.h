// Author: Tucker Haydon

#pragma once

#include <algorithm>

#include "line2d.h"
#include "polygon.h"
#include "line2d_potential.h"
#include "point2d_potential.h"
#include "potential2d.h"

namespace mediation_layer {
  class PolygonPotential : public Potential2D {
    public:
      struct Options {
        double activation_dist = 1.0;
        double min_dist = 0.5;
        double scale = 0.1;
      };

      PolygonPotential(const Polygon& polygon, 
                      const Options& options)
        : polygon_(polygon),
          options_(options) {
            Line2DPotential::Options edge_options;
            edge_options.min_dist = this->options_.min_dist;
            edge_options.activation_dist = this->options_.activation_dist;
            edge_options.scale = this->options_.scale;

            for(const Line2D& edge: this->polygon_.Edges()) {
              this->edge_potentials_.push_back(std::make_shared<Line2DPotential>(edge, edge_options));
            }

            Point2DPotential::Options vertex_options;
            vertex_options.min_dist = this->options_.min_dist;
            vertex_options.activation_dist = this->options_.activation_dist;
            vertex_options.scale = this->options_.scale;

            for(const Point2D& vertex: this->polygon_.Vertices()) {
              this->vertex_potentials_.push_back(std::make_shared<Point2DPotential>(vertex, vertex_options));
            }
      }

      Vec2D Resolve(const Point2D& point) const;

    private:
      Polygon polygon_;
      std::vector<std::shared_ptr<Line2DPotential>> edge_potentials_;
      std::vector<std::shared_ptr<Point2DPotential>> vertex_potentials_;
      Options options_;
      friend class PolygonPotentialView;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  Vec2D PolygonPotential::Resolve(const Point2D& point) const {
    return 
      std::accumulate(
        this->edge_potentials_.begin(),
        this->edge_potentials_.end(),
        Vec2D(0,0),
        [&](const Vec2D& current, const std::shared_ptr<Line2DPotential>& potential) {
          return current + potential->Resolve(point);
        }) 
        +
     std::accumulate(
        this->vertex_potentials_.begin(),
        this->vertex_potentials_.end(),
        Vec2D(0,0),
        [&](const Vec2D& current, const std::shared_ptr<Point2DPotential>& potential) {
          return current + potential->Resolve(point);
        });
  } 
}
