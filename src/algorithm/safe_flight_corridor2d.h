// Author: Tucker Haydon

#ifndef PATH_PLANNING_ALGORITHM_SAFE_FLIGHT_CORRIDOR2D
#define PATH_PLANNING_ALGORITHM_SAFE_FLIGHT_CORRIDOR2D

#include <vector>

#include "polygon.h"
#include "point2d.h"
#include "map2d.h"

namespace path_planning {
  class SafeFlightCorridor2D {
    private:
      Map2D map_;

    public:
      SafeFlightCorridor2D(){}

      bool SetMap(const Map2D& map);

      std::vector<Polygon> Run(const std::vector<Point2D>& path) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool SafeFlightCorridor2D::SetMap(const Map2D& map) {
    this->map_ = map;
    return true;
  }
}

#endif
