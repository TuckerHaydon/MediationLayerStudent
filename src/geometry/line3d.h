// Author: Tucker Haydon

#pragma once 

#include "types.h"

namespace path_planning {
  /*
   * Encapsulates information about a 3D line
   */
  class Line3D {
    private:
      Point3D start_;
      Point3D end_;

    public:
      Line3D(const Point3D& start = Point3D(), 
             const Point3D& end = Point3D())
        : start_(start),
          end_(end) {}

      const Point3D& Start() const;
      const Point3D& End() const;

      bool SetStart(const Point3D& start);
      bool SetEnd(const Point3D& end);

      Point3D AsVector() const;

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const Point3D& Line3D::Start() const {
    return this->start_;
  }

  inline const Point3D& Line3D::End() const {
    return this->end_;
  }

  inline bool Line3D::SetStart(const Point3D& start) {
    this->start_ = start;
    return true;
  }

  inline bool Line3D::SetEnd(const Point3D& end) {
    this->end_ = end;
    return true;
  }

  inline Point3D Line3D::AsVector() const {
    return this->end_ - this->start_;
  }
}
