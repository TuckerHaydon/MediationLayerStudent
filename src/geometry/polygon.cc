// Author: Tucker Haydon

#include "polygon.h"

#include <numeric>
#include <iostream>
#include <cmath>
#include <queue>

namespace path_planning { 
  Polygon Polygon::Expand(double dist) const { 
    this->Check();

    // Find the arithmetic mean of the vertices. For a convex polygon,
    // guaranteed to be contained by the polygon.
    double avg_x{0}, avg_y{0};
    for(const Point2D& point: this->vertices_) {
      avg_x += point.x();
      avg_y += point.y();
    }
    avg_x /= this->vertices_.size();
    avg_y /= this->vertices_.size();

    // Push every point out by dist * sgn(vertex - center_point)
    std::vector<Line2D> new_edges;
    new_edges.reserve(this->edges_.size());
    for(const Line2D& edge: this->edges_) { 
      const double dx_start = edge.Start().x() - avg_x;
      const double dy_start = edge.Start().y() - avg_y;
      const Point2D center_to_start(
          dist * ( (dx_start > 0) - (0 > dx_start)),
          dist * ( (dy_start > 0) - (0 > dy_start))
          );

      const double dx_end = edge.End().x() - avg_x;
      const double dy_end = edge.End().y() - avg_y;
      const Point2D center_to_end(
          dist * ( (dx_end > 0) - (0 > dx_end)),
          dist * ( (dy_end > 0) - (0 > dy_end))
          );

      const Point2D start = edge.Start() + center_to_start;
      const Point2D end = edge.End() + center_to_end;
      new_edges.emplace_back(start, end);
    }

    return Polygon(new_edges);
  }

  Polygon Polygon::Shrink(double dist) const {
    return this->Expand(-dist);
  }

  bool Polygon::IsConvex() const {
    for(size_t idx = 0; idx < this->edges_.size(); ++idx) {
      const Line2D& a = this->edges_[idx];
      const Line2D& b = this->edges_[(idx + 1) % this->edges_.size()];
      if(false == a.OnLeftSide(b.End())) {
        return false;
      }
    }

    return true;
  }

  bool Polygon::ConvexHullFromPoints(const std::vector<Point2D>& points) {
    std::vector<Point2D> vertices;
    
    Point2D left_most_point;
    double min_x = std::numeric_limits<double>::max();
    for(const Point2D& point: points) {
      if(point.x() < min_x) {
        left_most_point = point;
        min_x = point.x();
      }
    }
    vertices.push_back(left_most_point);

    Point2D from = left_most_point;
    while(true) {
      for(const Point2D& to: points) {
        if(from == to) { 
          continue; 
        }
        Line2D line(from, to);
        bool all_points_left = true;
        for(const Point2D& other: points) {
          if(false == line.OnLeftSide(other)) {
            all_points_left = false;
            break;
          }
        }
        if(true == all_points_left) {
          vertices.push_back(to);
          from = to;
          break;
        }
      }
      if(left_most_point == vertices.back()) { break; }
    }

    std::vector<Line2D> edges;
    for(size_t idx = 0; idx < vertices.size(); ++idx) {
      edges.emplace_back(vertices[idx], vertices[(idx+1) % vertices.size()]);
    }

    *(this) = Polygon(edges);
    return true;
  }
}
