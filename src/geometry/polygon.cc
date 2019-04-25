// Author: Tucker Haydon

#include "polygon.h"

namespace game_engine {
  Polygon Polygon::Expand(double dist) const { 
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

  void Polygon::ConvexHullFromPoints(const std::vector<Point2D>& points) {
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
  }

  Polygon::Polygon(const std::vector<Line2D>& edges)
    : edges_(edges) {
      this->UpdateVertices(); 
  }

  void Polygon::UpdateVertices() {
    this->vertices_.clear();
    this->vertices_.reserve(this->edges_.size());
    for(const Line2D& edge: this->edges_) {
      this->vertices_.push_back(edge.Start());
    }
  }

  const std::vector<Line2D>& Polygon::Edges() const {
    return this->edges_;
  }

  void Polygon::SetEdges(const std::vector<Line2D>& edges) {
    this->edges_ = edges;
    this->UpdateVertices();
  }

  const std::vector<Point2D>& Polygon::Vertices() const {
    return this->vertices_;
  }

  bool Polygon::Contains(const Point2D& point) const {
    for(const Line2D& edge: this->edges_) {
      if(false == edge.OnLeftSide(point)) { return false; }
    }

    return true;
  }

  Polygon Polygon::BoundingBox() const {
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

  void Polygon::ConstructFromPoints(const std::vector<Point2D>& points) {
    std::vector<Line2D> edges;
    edges.reserve(points.size());
    for(size_t idx = 0; idx < points.size(); ++idx) {
      edges.emplace_back(points[idx], points[(idx + 1) % points.size()]);
    }
    *(this) = Polygon(edges);
  }
  
  std::vector<Line2D> Polygon::ReversedEdges() const {
    std::vector<Line2D> reversed_edges;
    for(const Line2D& edge: this->edges_) {
      reversed_edges.emplace_back(edge.End(), edge.Start());
    }

    return reversed_edges;
  }

  Polygon Polygon::Invert() const {
    std::vector<Line2D> edges = this->edges_;
    std::reverse(edges.begin(), edges.end());
    std::vector<Line2D> reversed_edges;
    for(const Line2D& edge: edges) {
      reversed_edges.emplace_back(edge.End(), edge.Start());
    }
    return Polygon(reversed_edges);
  }
}
