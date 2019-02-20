// Author: Tucker Haydon

#pragma once

#include <memory>

#include "state2d.h"
#include "map2d.h"

namespace mediation_layer {
  // 2D mediation layer (ML). The ML should run as an independent thread. The ML
  // reads from proposed_state and modifies the trajectories to adhere to the
  // map and simulation constraints. The modifications are written to
  // updated_state.
  class MediationLayer2D {
    private:
      Map2D map_;
      volatile bool ok_{true};

    public:
      MediationLayer2D(const Map2D& map)
        : map_(map){}

      bool Run(std::shared_ptr<State2D> proposed_state, std::shared_ptr<State2D> updated_state);
      bool Stop();
  };
};
