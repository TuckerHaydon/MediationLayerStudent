// Author: Tucker Haydon

#pragma once

#include <utility>

#include "types.h"

namespace path_planning {
  /*
   * Encapsulates information about a 2D line
   */
  class Line2D {
    private:
      Point2D start_;
      Point2D end_;

    public:
      Line2D(const Point2D& start = Point2D(),
             const Point2D& end = Point2D())
        : start_(start),
          end_(end) {};

      const Point2D& Start() const;
      const Point2D& End() const;

      bool SetStart(const Point2D& start);
      bool SetEnd(const Point2D& end);
      
      // Express the line as a 2D vector.
      Vec2D AsVector() const;

      // Return the unit vector from start to end
      Vec2D AsUnitVector() const;

      // Returns a unit vector orthogonal to the line on the left side of the
      // line
      Vec2D OrthogonalUnitVector() const;

      // Determines if a point is on the left side of the line. The left side is
      // viewed from the starting point looking towards the ending point.
      // Succinctly represented as the cross product of this line and line
      // between the start point and the point in question. A point on the line
      // is considered on the left side.
      bool OnLeftSide(const Point2D& point) const;

      // Returns the standard form of the line: AX = B --> pair<A,B>
      std::pair<Point2D, double> StandardForm() const;

      // Returns the point of intersection between two lines provided they're
      // not parallel. X = inv([A1;A2]) * [B1;B2]
      Point2D IntersectionPoint(const Line2D& other) const;
      Point2D IntersectionPoint(const std::pair<Point2D, double>& other_sf) const;

      // Returns the point of intersection between this line and a line normal
      // to this line passing through a specified point
      Point2D NormalIntersectionPoint(const Point2D point) const;

      // Determines if a point lies on the line between the start and the end
      bool Contains(const Point2D& point) const;

      // Determines if a point projected onto the line is between the start and
      // end point
      bool ProjectedContains(const Point2D& point) const;
  };
}

