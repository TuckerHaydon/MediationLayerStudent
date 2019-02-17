// Author: Tucker Haydon

#pragma once

#include <memory>
#include <atomic>

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
      std::atomic<bool> ok_{true};

    public:
      MediationLayer2D(const Map2D& map)
        : map_(map){}

      bool Run(State2D& proposed_state, State2D& updated_state);
      bool Stop();
  };
};
