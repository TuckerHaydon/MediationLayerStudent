// Author: Tucker Haydon

#include "line2d.h"

namespace path_planning {
  const Point2D& Line2D::Start() const {
    return this->start_;
  }

  const Point2D& Line2D::End() const {
    return this->end_;
  }

  bool Line2D::SetStart(const Point2D& start) {
    this->start_ = start;
    return true;
  }

  bool Line2D::SetEnd(const Point2D& end) {
    this->end_ = end;
    return true;
  }

  Vec2D Line2D::AsVector() const {
    return this->end_ - this->start_;
  }

  Vec2D Line2D::AsUnitVector() const {
    return this->AsVector().normalized();
  }

  Vec2D Line2D::OrthogonalUnitVector() const {
    const Point2D unit = this->AsUnitVector();
    return Point2D(-unit.y(), unit.x());
  }

  bool Line2D::OnLeftSide(const Point2D& point) const {
    const Point2D p1 = this->AsVector();
    const Point2D p2(point - this->start_);
    return 
      (p1.x()*p2.y() - p1.y()*p2.x() >= 0);
  }

  std::pair<Point2D, double> Line2D::StandardForm() const {
    const Point2D vec = this->AsVector();
    const Point2D A(-vec.y(), vec.x());
    const double B = this->start_.dot(A);
    return std::pair<Point2D, double>(A,B);
  }

  Point2D Line2D::IntersectionPoint(const std::pair<Point2D, double>& other_sf) const {
    const std::pair<Point2D, double> this_sf = this->StandardForm();
    return (
        Eigen::Matrix<double,2,2>() << 
          this_sf.first.transpose(), 
          other_sf.first.transpose()
        ).finished().inverse() 
      * Eigen::Vector2d(this_sf.second, other_sf.second);
  }

  Point2D Line2D::IntersectionPoint(const Line2D& other) const {
    return this->IntersectionPoint(other.StandardForm());
  }

  Point2D Line2D::NormalIntersectionPoint(const Point2D point) const {
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

  bool Line2D::Contains(const Point2D& point) const {
    return 
      std::abs((point - this->start_).normalized().dot(this->AsUnitVector()) - 1) < 1e-3 &&
      std::abs((point - this->end_).normalized().dot(-this->AsUnitVector()) - 1) < 1e-3;
  }

  bool Line2D::ProjectedContains(const Point2D& point) const {
    const Point2D projected_point = (point - this->start_).dot(this->AsUnitVector()) *
      this->AsUnitVector() + this->start_;
    return this->Contains(projected_point);
  }
}
