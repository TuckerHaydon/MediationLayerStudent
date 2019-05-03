// Author: Tucker Haydon

#pragma once

#include <vector>
#include <string>
#include <memory>

#include "quad_state_warden.h"
#include "map3d.h"
#include "quad_state_watchdog_status.h"

namespace game_engine {
  // The state watchdog watches the positions of the quadcopters and determines
  // if any of them have flown too close to any obstacles. If they have, a
  // StateWatchdogStatus instance is set to report this violation
  class QuadStateWatchdog {
    public:
      struct Options {
        // Minimum l-infinity distance from all obstacles that a quad may fly
        double min_distance = 0.20;

        Options() {}
      };

      QuadStateWatchdog(const Options& options = Options())
        : options_(options) {}

      // Main thread function
      void Run(
          std::shared_ptr<QuadStateWarden> quad_state_warden,
          const std::vector<std::string>& quad_names,
          std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
          const Map3D map);

      // Stop this thread
      void Stop();

    private:
      volatile std::atomic_bool ok_{true};
      Options options_;

  };
}
