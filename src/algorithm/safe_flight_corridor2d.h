// Author: Tucker Haydon

#pragma once

#include <vector>

#include "polygon.h"
#include "map2d.h"

namespace mediation_layer {
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
