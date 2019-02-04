// Author: Tucker Haydon

// #include <queue>
#include <vector>
#include <algorithm>

#include "safe_flight_corridor2d.h"
#include "linear_constraint2d.h"

using namespace geometry;

namespace path_planning {
  namespace {
    double Distance(const Point2D& a, const Point2D& b) {
      return (a - b).norm();
    }

    Point2D FindClosestPoint(const Point2D& center, 
                             const Eigen::Matrix<double, 2, 2>& R, 
                             const double a_prime,
                             const Map2D& map,
                             const std::vector<LinearConstraint2D>& linear_constraints) {
      struct CandidatePoint {
        Point2D point_;
        double distance_;

        CandidatePoint(const Point2D& point, const double distance)
          : point_(point),
            distance_(distance) {}

        bool operator>(const CandidatePoint& rhs) const {
          return this->distance_ > rhs.distance_;
        }
      };

      // std::priority_queue<
      //   CandidatePoint, 
      //   std::vector<CandidatePoint>, 
      //   std::greater<CandidatePoint>> pq;
      std::vector<CandidatePoint> pq;
      std::vector<Polygon> obstacles = map.Obstacles();
      obstacles.push_back(map.Boundary());
      for(const Polygon& obstacle: obstacles) {
        for(const Line2D& edge: obstacle.Edges()) {
          // If the edge lies along a constraint, ignore it
          bool consider_edge = true;
          for(const LinearConstraint2D& lc: linear_constraints) {
            if(lc.ConstrainsEqual(edge.Start()) && lc.ConstrainsEqual(edge.End())) {
              consider_edge = false;
            }
          }
          if(false == consider_edge) { continue; }

          // Consider the point where the ellipse will intersect the line.
          // This can be thought of as finding the y semi-major axis with a length
          // such that the intersection of the line and the ellipse has one
          // repeated solution. Plug the line equation into the ellipse equation
          // and require the discriminant to be zero. 
          //
          // Given y=mx+c and x^2/a^2 + y^2/b^2 = 1,
          //       b = sqrt(c^2 - a^2 * m^2)
          //       x = (-a^2 * m * c) / (b^2 + a^2 * m^2)
          //       y = mx + c
          //
          // Note that is solution requires a coordinate transform of the line to
          // a coordinate system that is centered at the ellipse center and
          // aligned with it's XY axes.

          // R:  ellipse -> global
          // R': global -> ellipse

          const std::pair<Eigen::Vector2d, double> AB = edge.StandardForm();
          const std::pair<Eigen::Vector2d, double> AB_tilde = 
            std::pair<Eigen::Vector2d, double>(
              R.transpose() * AB.first,
              AB.second - AB.first.transpose() * center);

          const double a = a_prime;
          const double m = -AB_tilde.first(0) / AB_tilde.first(1);
          const double c = AB_tilde.second / AB_tilde.first(1);
          const double b = std::sqrt(c*c - a*a * m*m);
          // const double a = a_prime;
          // const double m = -AB.first(0) / AB.first(1);
          // const double c = AB.second / AB.first(1);
          // const double b = std::sqrt(c*c - a*a * m*m);

          // x_tilde,y_tilde are in ellipse space
          const double x_tilde = (-a*a*m*c) / (b*b + a*a * m*m);
          const double y_tilde = m * x_tilde + c;

          // Transform back to global space
          const Point2D intersection_point = 
            R * Eigen::Vector2d(x_tilde, y_tilde) + center;
          // const Point2D intersection_point(x_tilde, y_tilde);

          // The start and end vertices are automatically in the expansion path
          // since the implicit constrains reduce the space to only
          // vertices/polygons within the expansion path. Need to add the
          // vertices to account for the case where the ellipse will intersect a
          // linear constraint outside the line. When that occurs, the
          // intersection point is tossed, but it still might intersect the
          // vertex. 
          const Point2D start_ellipse = R.transpose() * (edge.Start() - center);
          const Point2D end_ellipse = R.transpose() * (edge.End() - center);
          const double distance_to_start = 
            std::sqrt((a*a * std::pow(start_ellipse.y(), 2)) 
                / (a*a - std::pow(start_ellipse.x(), 2)));
          pq.emplace_back(edge.Start(), distance_to_start);
          const double distance_to_end = 
            std::sqrt((a*a * std::pow(end_ellipse.y(), 2)) 
                / (a*a - std::pow(end_ellipse.x(), 2)));
          pq.emplace_back(edge.End(), distance_to_end);
          // const double distance_to_start = 
          //   std::sqrt((a*a * std::pow(edge.Start().y(), 2)) 
          //       / (a*a - std::pow(edge.Start().x(), 2)));
          // pq.emplace(edge.Start(), distance_to_start);
          // const double distance_to_end = 
          //   std::sqrt((a*a * std::pow(edge.End().y(), 2)) 
          //       / (a*a - std::pow(edge.End().x(), 2)));
          // pq.emplace(edge.End(), distance_to_end);

          // If the intersection point does not lie between the two vertices,
          // ignore it
          if(Distance(intersection_point, edge.Start()) > Distance(edge.Start(), edge.End()) ||
             Distance(intersection_point, edge.End())   > Distance(edge.Start(), edge.End())) {
            continue;
          }

          pq.emplace_back(intersection_point, b);
        }
      }
      return std::min_element(pq.begin(), pq.end(), 
          [](const CandidatePoint& lhs, const CandidatePoint& rhs){
            return lhs.distance_ < rhs.distance_;
          })->point_;
      // return pq.top().point_;
    }

    Map2D UpdateMap(const LinearConstraint2D& lc,
                    const Map2D& map) {

      // Update the obstacles
      std::vector<Polygon> obstacles = map.Obstacles();
      obstacles.push_back(map.Boundary());
      std::vector<Polygon> new_obstacles; 
      for(const Polygon& obstacle: obstacles)  {
        std::vector<Point2D> new_vertices;
        // Edges are counter-clockwise order
        for(const Line2D& edge: obstacle.Edges()) {

          const bool constrains_start = lc.Constrains(edge.Start());
          const bool constrains_end = lc.Constrains(edge.End());

          // If neither vertex meets constraint, neither need to be considered
          // in the future
          if(false == constrains_start &&
             false == constrains_end) { continue; }

          // If start vertex meets the constraint, add it
          if(true == constrains_start) { new_vertices.push_back(edge.Start()); }

          // If the constraint intersects the edge, the intersection point needs
          // to be considered in the future
          if(true == (constrains_start ^ constrains_end)) {
           new_vertices.push_back(
               edge.IntersectionPoint(std::pair<Point2D, double>(lc.A_, lc.B_)));
          }
        }

        // Degenerate case where all vertices are the same point
        bool degenerate_obstacle = true;
        if(0 != new_vertices.size()) {
          for(const Point2D& vertex: new_vertices) {
            if(false == new_vertices[0].isApprox(vertex, 1e-3)) {
              degenerate_obstacle = false;
              break;
            }
          }
        }

        if(false == degenerate_obstacle) {
          Polygon new_obstacle; 
          new_obstacle.ConstructFromPoints(new_vertices);
          new_obstacles.push_back(new_obstacle);
        }
      }

      const Polygon new_boundary = new_obstacles.back();
      new_obstacles.pop_back();

      return Map2D(new_boundary, new_obstacles);
    }
  }

  
  std::vector<Polygon> SafeFlightCorridor2D::Run(const std::vector<Point2D>& path) const {
    /* ALGORITHM
     * For every line:
     *  1) Initialize ellipse at midpoint of line. x-axis is aligned with line.
     *  2) Find the point corresponding to the closest obstacle to the center of
     *     the ellipse, p*. The plane defined between this point and the x-axis is
     *     the xy plane.
     *  3) Take the cross product between the x-axis and the z-axis to define
     *     the y-axis.
     *  4) Find the distance between the center point and the end point. This
     *     defines the length of the semi-major axis for the x-axis, a'.
     *  5) Project the contact point onto the x- and y-axes. Define these as x',
     *     y'.
     *  6) x',y' satisfy the ellipse equation: ((x' - x_c)/a')^2 + ((y' -
     *     y_c)/b')^2 = 1. Solve for b'.
     *  7) Create the ellipse matrix: E=RSR'. S=diag(a',b'), R = [x-axis, y-axis].
     *  8) Define a tangent line at p*: w'p* < v. 
     *     w = 2 * inv(E) * inv(E)' * (p* - [x_c;y_c])
     *  9) Remove/Modify all points/obstacles that are on the wrong side of the
     *     tangent line. Polygon lines can be modified by looking for the point
     *     of intersection between the polygon line and the tangent line:
     *     p^ = inv(a_1'; a_2') * [b_1;b_2]
     *  10) Goto 2 and repeat until there are no more obstacles.
     *  11) Create convex polygon from intersection points between tangent lines
     */

    std::vector<Polygon> safe_flight_corridor;
    safe_flight_corridor.reserve(path.size() - 1);

    // For every line
    // TODO: Parallelize this
    for(size_t idx = 0; idx < path.size()-1; ++idx) {
      // Line segment
      const Point2D& start = path[idx];
      const Point2D& end = path[idx + 1];
      const Point2D center = (end + start) / 2;

      // x-axis
      const Point2D x_axis = (end - center).normalized();
      const double a_prime = (end - center).norm();
  
      // y-axis
      const Eigen::Vector3d y_axis3D = Eigen::Vector3d(0,0,1).cross(Eigen::Vector3d(x_axis.x(), x_axis.y(), 0));
      const Point2D y_axis(y_axis3D.x(), y_axis3D.y());

      // Ellipse Rotation matrix
      const Eigen::Matrix<double, 2, 2> R = 
        (Eigen::Matrix<double, 2, 2>() << x_axis, y_axis).finished();

      // Make a mutable copy of the map
      Map2D current_map = this->map_;

      // Shift everything in the map into ellipse coordinates
      // {
      //   std::vector<Line2D> new_boundary_edges;
      //   new_boundary_edges.reserve(current_map.Boundary().Edges().size());
      //   for(const Line2D& edge: current_map.Boundary().Edges()) {
      //     new_boundary_edges.emplace_back(
      //         R.transpose() * (edge.Start() - center),
      //         R.transpose() * (edge.End() - center));  
      //   }
      //   const Polygon new_boundary(new_boundary_edges);

      //   std::vector<Polygon> new_obstacles;
      //   for(const Polygon& obstacle: current_map.Obstacles()) {
      //     std::vector<Line2D> new_obstacle_edges;
      //     new_obstacle_edges.reserve(obstacle.Edges().size());
      //     for(const Line2D& edge: obstacle.Edges()) {
      //       new_obstacle_edges.emplace_back(
      //           R.transpose() * (edge.Start() - center),
      //           R.transpose() * (edge.End() - center));  
      //     }
      //     new_obstacles.emplace_back(new_obstacle_edges);
      //   }

      //   current_map = Map2D(new_boundary, new_obstacles);
      // }

      std::vector<LinearConstraint2D> linear_constraints;
      { // Implicit linear constraints at extents of ellipse
        const Line2D l(start, end);
        const std::pair<Point2D, double> sf = l.StandardForm();

        const Point2D A_start(-sf.first(1), sf.first(0));
        const double B_start = A_start.dot(start);
        const LinearConstraint2D l_start(A_start, B_start);
        linear_constraints.push_back(l_start);
        current_map = UpdateMap(l_start, current_map);

        const Point2D A_end(-sf.first(1), sf.first(0));
        const double B_end = A_end.dot(end);
        const LinearConstraint2D l_end(A_end, B_end);
        linear_constraints.push_back(l_end);
        current_map = UpdateMap(l_end, current_map);
      }

      // Iterate until no obstacles exist within space
      while(true) { 

        // Find the closest point to the center of the ellipse
        const Point2D closest_point = FindClosestPoint(center, R, a_prime, current_map, linear_constraints);
        const double x_prime = (closest_point - center).dot(x_axis);
        const double y_prime = (closest_point - center).dot(y_axis);
        const double b_prime = std::sqrt((std::pow(y_prime,2)) / (1 - std::pow(x_prime/a_prime,2)));
  
        // Ellipse matrices
        const Eigen::Matrix<double, 2, 2> S = Eigen::Vector2d(a_prime, b_prime).asDiagonal();
        const Eigen::Matrix<double, 2, 2> E = R * S * R.transpose();
  
        // Tangent line segment
        const Eigen::Matrix<double, 2, 2> E_inv = E.inverse();
        const Eigen::Vector2d A_prime = 2 * E_inv * E_inv.transpose() * (closest_point - center);
        const double B_prime = A_prime.dot(closest_point);
        const LinearConstraint2D lc(A_prime, B_prime);
        linear_constraints.push_back(lc);
  
        // Update map. Only keep points that satisfy A_prime * p < B_prime
        current_map = UpdateMap(lc, current_map);

        // If there are still obstacles in the vicinity, continue
        // if(0 != current_map.Obstacles().size()) { continue; }
        bool all_obstacles_constrained = true;
        for(const Polygon& obstacle: current_map.Obstacles()) {
          bool is_obstacle_constrained = true;
          for(const Point2D& vertex: obstacle.Vertices()) {
            bool is_vertex_constrained = false;
            for(const LinearConstraint2D& lc_: linear_constraints) {
              if(true == lc_.ConstrainsEqual(vertex)) { is_vertex_constrained = true; break; }
            }
            if(false == is_vertex_constrained) { is_obstacle_constrained = false; break; }
          }
          if(false == is_obstacle_constrained) { all_obstacles_constrained = false; break; }
        }
        if(false == all_obstacles_constrained) { continue; }

        // If there are still boundaries in the vicinity, continue
        bool is_boundary_constrained = true;
        for(const Point2D& vertex: current_map.Boundary().Vertices()) {
          bool is_vertex_constrained = false;
          for(const LinearConstraint2D& lc_: linear_constraints) {
            if(true == lc_.ConstrainsEqual(vertex)) { is_vertex_constrained = true; break; }
          }
          if(false == is_vertex_constrained) { is_boundary_constrained = false; break; }
        }
        if(false == is_boundary_constrained) { continue; }

        // Exit conditions have been met
        break;
      }
      
      // Shift everything in the map back to the map coordinates
      // {
      //   std::vector<Line2D> new_boundary_edges;
      //   new_boundary_edges.reserve(current_map.Boundary().Edges().size());
      //   for(const Line2D& edge: current_map.Boundary().Edges()) {
      //     new_boundary_edges.emplace_back(
      //         R * (edge.Start()) + center,
      //         R * (edge.End()) + center);  
      //   }
      //   const Polygon new_boundary(new_boundary_edges);

      //   std::vector<Polygon> new_obstacles;
      //   for(const Polygon& obstacle: current_map.Obstacles()) {
      //     std::vector<Line2D> new_obstacle_edges;
      //     new_obstacle_edges.reserve(obstacle.Edges().size());
      //     for(const Line2D& edge: obstacle.Edges()) {
      //       new_obstacle_edges.emplace_back(
      //           R * edge.Start() + center,
      //           R * edge.End() + center);  
      //     }
      //     new_obstacles.emplace_back(new_obstacle_edges);
      //   }

      //   current_map = Map2D(new_boundary, new_obstacles);
      // }

      std::cout << "Map vertices: " << std::endl;
      for(const Point2D& vertex: current_map.Boundary().Vertices()) {
        std::cout << vertex.transpose() << std::endl;
      }
      safe_flight_corridor.push_back(current_map.Boundary());
    }

    return safe_flight_corridor;
  }

}
