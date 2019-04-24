// Author: Tucker Haydon

#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

#include "quad_state_warden.h"
#include "quad_state_guard.h"
#include "quad_state.h"

namespace mediation_layer {
  // Class that manages data transfer between a state warden and state guards
  class QuadStateDispatcher {
    private:
      volatile std::atomic_bool ok_{true};

      // Function that each thread in the pool will run
      //
      // Note that values are intentionally copied
      void AwaitStateChange(
          const std::string key, 
          std::shared_ptr<QuadStateWarden> warden, 
          std::shared_ptr<QuadStateGuard> guard);

    public:
      // Start all the threads in the pool and wait for them to finish. This
      // function should only be called after the QuadStateWarden has been
      // completely set up. New registrations in the QuadStateWarden will not
      // be seen by the QuadStateDispatcher, and thus will not be dispatched.
      //
      // Note that values are intentionally copied
      void Run(
          std::shared_ptr<QuadStateWarden> warden, 
          std::unordered_map<std::string, std::shared_ptr<QuadStateGuard>> state_guards);

      // Stop this thread and all the sub-threads
      void Stop();
  };
}
