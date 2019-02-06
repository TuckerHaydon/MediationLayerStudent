// Author: Tucker Haydon

#ifndef GEOMETRY_POLYGON_H
#define GEOMETRY_POLYGON_H

#include <vector>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "line2d.h"
#include "point2d.h"

namespace path_planning{
  // Class representing a 2D convex polygon.
  class Polygon {
    private:
      std::vector<Line2D> edges_;
      std::vector<Point2D> vertices_;

      bool UpdateVertices();

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

      bool ConstructFromPoints(const std::vector<Point2D>& points);

      void Check() const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
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
}
#endif
