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

namespace game_engine{
  // Implementation of a 2D closed, convex polygon in R2. A polygon may be
  // represented by a set of continuous 2D lines all lying on the same plane and
  // encapsulating a convex region. 
  //
  // Edges must be specified with the following properties:
  //  1) The end point of each edge must coincide with the start-point of
  //     another edge
  //  2) Edges must be ordered such that the cross product between sequential
  //     edges points towards the center of the convex region
  //
  // TODO: Add checks for convex and closed properties
  class Polygon {
    private:
      // Edges of the polygon
      std::vector<Line2D> edges_;

      // Vertices of the polygon
      std::vector<Point2D> vertices_;

      // Updates vertices vector to reflect changes in edges
      void UpdateVertices();

      // Forward-declare parser
      friend class YAML::convert<Polygon>;

    public:
      // Constructor
      Polygon(const std::vector<Line2D>& edges = {});

      // Edges accessor
      const std::vector<Line2D>& Edges() const;

      // Edges setter
      void SetEdges(const std::vector<Line2D>& edges);

      // Vertices accessor
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
      void ConvexHullFromPoints(const std::vector<Point2D>& points);

      // Constructs the polygon from a list of points. When traversed, the list
      // of points should form a convex polygon.
      void ConstructFromPoints(const std::vector<Point2D>& points);

      // Reverse the start and end points of all the edges to reverse the
      // direction of the edges. 
      std::vector<Line2D> ReversedEdges() const;

      // Inverts a polygon. Returns a polygon whose edge order and node order
      // are reversed. Edges will now point outwards. Functions that assume
      // edge order will not work as expected.
      Polygon Invert() const;
  };
}

namespace YAML {
  template<>
  struct convert<game_engine::Polygon> {
    static Node encode(const game_engine::Polygon& rhs) {
      Node node;
      node.push_back(rhs.edges_);
      return node;
    }
  
    static bool decode(const Node& node, game_engine::Polygon& rhs) {
      if(!node.IsSequence()) {
        return false;
      }
  
      std::vector<game_engine::Line2D> edges;
      edges.reserve(node.size());
      for(size_t idx = 0; idx < node.size(); ++idx) {
        edges.push_back(node[idx].as<game_engine::Line2D>());
      }
  
      rhs.SetEdges(edges);
      return true;
    }
  };
}
