// Author: Tucker Haydon

#pragma once

#include <vector>
#include <cstdlib>
#include <numeric>
#include <iostream>
#include <cmath>
#include <queue>
#include <limits>

#include "line2d.h"
#include "yaml-cpp/yaml.h"

namespace mediation_layer{
  // Class representing a 2D convex polygon.
  class Polygon {
    private:
      std::vector<Line2D> edges_;
      std::vector<Point2D> vertices_;

      bool UpdateVertices();
      friend class YAML::convert<Polygon>;

    public:
      Polygon(const std::vector<Line2D>& edges = {});

      const std::vector<Line2D>& Edges() const;
      bool SetEdges(const std::vector<Line2D>& edges);

      const std::vector<Point2D>& Vertices() const;

      // Determines if a point is contained within the polygon. A point is
      // contained if, given a counter-clockwise ordering of all of the edges,
      // the point is on the left side of all of the edges
      bool Contains(const Point2D& point) const;

      // Determines if the polygon is convex
      bool IsConvex() const;

      // Expands a polygon's vertices by a specified distance away from the
      // center of the polygon. Both the x and y coordinates are shifted
      // equally.
      Polygon Expand(double dist) const;

      // Shrinks a polygon's vertices by a specified distance towards the center
      // of the polygon. Both the x and y coordinates are shifted equally.
      Polygon Shrink(double dist) const;

      // Returns a rectangle that bounds this polygon
      Polygon BoundingBox() const;

      // Constructs a convex polygon from the convex hull of a set of points.
      // Reference: https://en.wikipedia.org/wiki/Gift_wrapping_algorithm
      bool ConvexHullFromPoints(const std::vector<Point2D>& points);

      // Constructs the polygon from a list of points. When traversed, the list
      // of points should form a convex polygon.
      bool ConstructFromPoints(const std::vector<Point2D>& points);

      // Reverse the start and end points of all the edges to reverse the
      // direction of the edges. 
      std::vector<Line2D> ReversedEdges() const;

      // Inverts a polygon. Returns a polygon whose edge order and node order
      // are reversed. Edges will now point outwards. Functions that assume
      // edge order will not work as expected.
      Polygon Invert() const;

      // Check that the polygon is valid
      void Check() const;
  };

  //============================
  //     IMPLEMENTATION
  //============================
  inline Polygon Polygon::Expand(double dist) const { 
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

  inline Polygon Polygon::Shrink(double dist) const {
    return this->Expand(-dist);
  }

  inline bool Polygon::IsConvex() const {
    for(size_t idx = 0; idx < this->edges_.size(); ++idx) {
      const Line2D& a = this->edges_[idx];
      const Line2D& b = this->edges_[(idx + 1) % this->edges_.size()];
      if(false == a.OnLeftSide(b.End())) {
        return false;
      }
    }

    return true;
  }

  inline bool Polygon::ConvexHullFromPoints(const std::vector<Point2D>& points) {
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

  inline Polygon::Polygon(const std::vector<Line2D>& edges)
    : edges_(edges) {
      this->UpdateVertices(); 
  }

  inline bool Polygon::UpdateVertices() {
    this->vertices_.clear();
    this->vertices_.reserve(this->edges_.size());
    for(const Line2D& edge: this->edges_) {
      this->vertices_.push_back(edge.Start());
    }
    return true;
  }

  inline const std::vector<Line2D>& Polygon::Edges() const {
    return this->edges_;
  }

  inline bool Polygon::SetEdges(const std::vector<Line2D>& edges) {
    this->edges_ = edges;
    this->UpdateVertices();

    return true;
  }

  inline const std::vector<Point2D>& Polygon::Vertices() const {
    return this->vertices_;
  }

  inline bool Polygon::Contains(const Point2D& point) const {
    this->Check();

    for(const Line2D& edge: this->edges_) {
      if(false == edge.OnLeftSide(point)) { return false; }
    }

    return true;
  }

  inline Polygon Polygon::BoundingBox() const {
    double min_x{std::numeric_limits<double>::max()},
           min_y{std::numeric_limits<double>::max()},
           max_x{std::numeric_limits<double>::min()},
           max_y{std::numeric_limits<double>::min()};

    for(const Point2D& point: this->vertices_) {
      if(point.x() < min_x) { min_x = point.x(); }
      if(point.x() > max_x) { max_x = point.x(); }
      if(point.y() < min_y) { min_y = point.y(); }
      if(point.y() > max_y) { max_y = point.y(); }
    }

    return Polygon({
      Line2D(Point2D(min_x, min_y), Point2D(max_x, min_y)),
      Line2D(Point2D(max_x, min_y), Point2D(max_x, max_y)),
      Line2D(Point2D(max_x, max_y), Point2D(min_x, max_y)),
      Line2D(Point2D(min_x, max_y), Point2D(min_x, min_y))});
  }

  inline bool Polygon::ConstructFromPoints(const std::vector<Point2D>& points) {
    std::vector<Line2D> edges;
    edges.reserve(points.size());
    for(size_t idx = 0; idx < points.size(); ++idx) {
      edges.emplace_back(points[idx], points[(idx + 1) % points.size()]);
    }
    *(this) = Polygon(edges);

    return true;
  }

  inline void Polygon::Check() const {  
    // TODO: Check that the polygon is complete
    if(false == this->IsConvex()) {
      std::cerr << "Polygon is not convex. Exiting." << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
  
  inline std::vector<Line2D> Polygon::ReversedEdges() const {
    std::vector<Line2D> reversed_edges;
    for(const Line2D& edge: this->edges_) {
      reversed_edges.emplace_back(edge.End(), edge.Start());
    }

    return reversed_edges;
  }

  inline Polygon Polygon::Invert() const {
    std::vector<Line2D> edges = this->edges_;
    std::reverse(edges.begin(), edges.end());
    std::vector<Line2D> reversed_edges;
    for(const Line2D& edge: edges) {
      reversed_edges.emplace_back(edge.End(), edge.Start());
    }
    return Polygon(reversed_edges);
  }
}

namespace YAML {
  template<>
  struct convert<mediation_layer::Polygon> {
    static Node encode(const mediation_layer::Polygon& rhs) {
      Node node;
      node.push_back(rhs.edges_);
      return node;
    }
  
    static bool decode(const Node& node, mediation_layer::Polygon& rhs) {
      if(!node.IsSequence()) {
        return false;
      }
  
      std::vector<mediation_layer::Line2D> edges;
      edges.reserve(node.size());
      for(size_t idx = 0; idx < node.size(); ++idx) {
        edges.push_back(node[idx].as<mediation_layer::Line2D>());
      }
  
      rhs.SetEdges(edges);
      return true;
    }
  };
}
