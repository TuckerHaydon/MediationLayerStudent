// Author: Tucker Haydon

#pragma once

#include "gnuplot-iostream.h"

namespace path_planning {
  class Potential2DView {
    public:
      virtual ~Potential2DView() = default;
      virtual Gnuplot& Display(Gnuplot& gp) const = 0;
  };
}
