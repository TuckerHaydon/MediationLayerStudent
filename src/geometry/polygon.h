// Author: Tucker Haydon

#pragma once

#include <vector>

#include "line2d.h"

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

      // Constructs the polygon from a list of points. When traversed, the list
      // of points should form a convex polygon.
      bool ConstructFromPoints(const std::vector<Point2D>& points);

      // Check that the polygon is valid
      void Check() const;
  };
}
