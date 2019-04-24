// Author: Tucker Haydon

#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <string>
#include <map>

#include "trajectory_warden.h"
#include "trajectory_publisher_node.h"

namespace mediation_layer {
  // Class that manages data transfer between a TrajectoryWarden and a
  // TrajectoryPublisherNode. This class maintains an internal thread pool that
  // await changes in TrajectoryOut. Once a change has been made, the thread
  // pushes the data to the TrajectoryPublisherNode to be pushed onto the
  // network.
  class TrajectoryDispatcher {
    private:
      volatile std::atomic_bool ok_{true};

      // Function that each thread in the pool will run
      //
      // Note that values are intentionally copied
      void AwaitTrajectoryChange(
          const std::string key, 
          std::shared_ptr<TrajectoryWarden> warden, 
          std::shared_ptr<TrajectoryPublisherNode> publisher);

    public:
      // Start all the threads in the pool and wait for them to finish. This
      // function should only be called after the TrajectoryWarden has been
      // completely set up. New registrations in the TrajectoryWarden will not
      // be seen by the TrajectoryDispatcher, and thus will not be dispatched.
      //
      // Note that values are intentionally copied
      void Run(
          std::shared_ptr<TrajectoryWarden> warden, 
          std::unordered_map<
            std::string, 
            std::shared_ptr<TrajectoryPublisherNode>> trajectory_publishers);

      // Stop this thread and all the sub-threads
      void Stop();
  };
}
