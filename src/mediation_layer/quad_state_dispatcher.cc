// Author: Tucker Haydon

#include "quad_state_dispatcher.h"

namespace game_engine {
  void QuadStateDispatcher::Run(
      std::shared_ptr<QuadStateWarden> warden, 
      std::unordered_map<std::string, std::shared_ptr<QuadStateGuard>> state_guards) {
    // Local thread pool 
    std::vector<std::thread> thread_pool;

    // Get all registered trajectories
    const std::set<std::string> state_keys = warden->Keys();

    // Assign a thread to await changes in each trajectory
    for(const std::string& key: state_keys) {
      thread_pool.push_back(
          std::move(
            std::thread([&](){
              this->AwaitStateChange(key, warden, state_guards[key]);
              })));
    }

    // Wait for this thread to receive a stop command
    std::thread kill_thread(
        [&, this]() {
          while(true) {
            if(false == this->ok_) {
              break;
            } else {
              std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
          }
        });

    kill_thread.join();

    // Wait for thread pool to terminate
    for(std::thread& t: thread_pool) {
      t.join();
    } 
  }

  void QuadStateDispatcher::AwaitStateChange(
      const std::string key, 
      std::shared_ptr<QuadStateWarden> warden, 
      std::shared_ptr<QuadStateGuard> guard) {
    while(this->ok_) {
      QuadState state;
      if(true == warden->Await(key, state)) {
        guard->Write(state);
      }
    }
  }

  void QuadStateDispatcher::Stop() {
    this->ok_ = false;
  }
}
