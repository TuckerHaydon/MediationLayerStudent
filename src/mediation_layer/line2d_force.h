// Author: Tucker Haydon

#ifndef PATH_PLANNING_MEDIATION_LAYER_LINE2D_FORCE_H
#define PATH_PLANNING_MEDIATION_LAYER_LINE2D_FORCE_H

#include "line2d.h"
#include "point2d.h"

namespace path_planning {
  class Line2DForce {
    private:
      Line2D& line_;
      double activation_dist_;
      bool activate_behind_;

    public:
      Line2DForce(const Line2D& line,
                  double activation_dist = 1.0, 
                  bool activate_behind = false)
        : line_(line),
          activation_dist_(activation_dist),
          activate_behind_(activate_behind) {}

      Point2D& Evaluate(const Point2D& point) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  Point2D& Line2DForce::Evaluate(const Point2D& point) const {
    // If the point if not within the orthogonal line bounds, no force
    if(false == this->line_.ProjectedContains(point))
    { return Point2D(0,0); }

    // If the point is on the right side of the line and the activation flag is
    // not set, no force
    if(false == this->activate_behind_ &&
       false == this->line_.OnLeftSide(point))
    { return Point2D(0,0); }


  }
  
}

#endif
