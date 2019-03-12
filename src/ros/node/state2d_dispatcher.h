// Author: Tucker Haydon

#pragma once

#include <vector>
#include <memory>
#include <thread>
#include <functional>
#include <atomic>
#include <ros/ros.h>

#include "publisher_guard.h"
#include "state2d.h"

namespace mediation_layer {
  namespace {
    bool DispatchFunction(
        std::string& key,
        std::shared_ptr<State2D>& state,
        std::shared_ptr<PublisherGuard<std_msgs::String>>& publisher,
        volatile std::atomic_bool& ok
        ) {

      Trajectory2D trajectory;
      while(true == ok) {
        // Await change in state
        // Compose message
        // Publish

        // state.Await(key, trajectory);
        // Construct message
        // publisher.Publish()
      } 
      return true;
    }
  }

  // Class that manages the dispatching of the substates of a State2D object.
  // Should be run as an independent thread. Managers a pool of threads --- one
  // for each substate. Each thread asynchonously waits for the state to be
  // modified and then pushes it to its corresponding publisher.
  class State2DDispatcher {
    private:
      std::shared_ptr<State2D> state_;
      std::vector<std::shared_ptr<PublisherGuard<std_msgs::String>>> publishers_;
      volatile std::atomic_bool ok_{true};

    public:
      State2DDispatcher(
          const std::shared_ptr<State2D> state,
          const std::vector<std::shared_ptr<ros::Publisher>> publishers) 
        : state_(state) {
          this->publishers_.clear();
          this->publishers_.reserve(publishers.size());
          for(const std::shared_ptr<ros::Publisher>& publisher: publishers) {
            this->publishers_.push_back(std::make_shared<PublisherGuard<std_msgs::String>>(publisher));
          }
        }

      bool Run();
      bool Stop();
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline bool State2DDispatcher::Run() {
    size_t num_threads = this->publishers_.size();
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    for(size_t idx = 0; idx < num_threads; ++idx) {
      std::string key = "";
      threads.push_back(std::move(std::thread(
              DispatchFunction, 
              std::ref(key),
              std::ref(this->state_), 
              std::ref(this->publishers_[idx]),
              std::ref(this->ok_)
              )));
    }

    for(size_t idx = 0; idx < num_threads; ++idx) {
      threads[idx].join();
    }

    return true;
  }

  inline bool State2DDispatcher::Stop() {
    this->ok_ = false;
    return true;
  }
}
