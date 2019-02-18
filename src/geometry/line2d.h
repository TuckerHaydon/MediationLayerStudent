// Author: Tucker Haydon

#pragma once

#include <utility>

#include "types.h"
#include "yaml-cpp/yaml.h"

namespace mediation_layer {
  /*
   * Encapsulates information about a 2D line
   */
  class Line2D {
    private:
      Point2D start_;
      Point2D end_;
      friend class YAML::convert<Line2D>;

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

      // Midpoint of the line
      Point2D Midpoint() const;
  };

  //============================
  //     IMPLEMENTATION
  //============================
  inline const Point2D& Line2D::Start() const {
    return this->start_;
  }

  inline const Point2D& Line2D::End() const {
    return this->end_;
  }

  inline bool Line2D::SetStart(const Point2D& start) {
    this->start_ = start;
    return true;
  }

  inline bool Line2D::SetEnd(const Point2D& end) {
    this->end_ = end;
    return true;
  }

  inline Vec2D Line2D::AsVector() const {
    return this->end_ - this->start_;
  }

  inline Vec2D Line2D::AsUnitVector() const {
    return this->AsVector().normalized();
  }

  inline Vec2D Line2D::OrthogonalUnitVector() const {
    const Point2D unit = this->AsUnitVector();
    return Point2D(-unit.y(), unit.x());
  }

  inline bool Line2D::OnLeftSide(const Point2D& point) const {
    const Point2D p1 = this->AsVector();
    const Point2D p2(point - this->start_);
    return 
      (p1.x()*p2.y() - p1.y()*p2.x() >= 0);
  }

  inline std::pair<Point2D, double> Line2D::StandardForm() const {
    const Point2D vec = this->AsVector();
    const Point2D A(-vec.y(), vec.x());
    const double B = this->start_.dot(A);
    return std::pair<Point2D, double>(A,B);
  }

  inline Point2D Line2D::IntersectionPoint(const std::pair<Point2D, double>& other_sf) const {
    const std::pair<Point2D, double> this_sf = this->StandardForm();
    return (
        Eigen::Matrix<double,2,2>() << 
          this_sf.first.transpose(), 
          other_sf.first.transpose()
        ).finished().inverse() 
      * Eigen::Vector2d(this_sf.second, other_sf.second);
  }

  inline Point2D Line2D::IntersectionPoint(const Line2D& other) const {
    return this->IntersectionPoint(other.StandardForm());
  }

  inline Point2D Line2D::NormalIntersectionPoint(const Point2D point) const {
    std::pair<Point2D, double> sf = this->StandardForm();
    const Point2D A_prime(-sf.first.y(), sf.first.x());
    const double B_prime = A_prime.dot(point);
    return (
        Eigen::Matrix<double,2,2>() << 
          sf.first.transpose(), 
          A_prime.transpose()
        ).finished().inverse() 
      * Eigen::Vector2d(sf.second, B_prime);
  }

  inline bool Line2D::Contains(const Point2D& point) const {
    return 
      std::abs((point - this->start_).normalized().dot(this->AsUnitVector()) - 1) < 1e-3 &&
      std::abs((point - this->end_).normalized().dot(-this->AsUnitVector()) - 1) < 1e-3;
  }

  inline bool Line2D::ProjectedContains(const Point2D& point) const {
    const Point2D projected_point = (point - this->start_).dot(this->AsUnitVector()) *
      this->AsUnitVector() + this->start_;
    return this->Contains(projected_point);
  }

  inline Point2D Line2D::Midpoint() const {
    return this->start_ + this->AsVector() / 2;
  }
}

namespace YAML {
template<>
struct convert<mediation_layer::Line2D> {
  static Node encode(const mediation_layer::Line2D& rhs) {
    Node node;
    node.push_back(rhs.start_.x());
    node.push_back(rhs.start_.y());
    node.push_back(rhs.end_.x());
    node.push_back(rhs.end_.y());
    return node;
  }

  static bool decode(const Node& node, mediation_layer::Line2D& rhs) {
    if(!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.start_.x() = node[0].as<double>();
    rhs.start_.y() = node[1].as<double>();
    rhs.end_.x()   = node[2].as<double>();
    rhs.end_.y()   = node[3].as<double>();
    return true;
  }
};
}
