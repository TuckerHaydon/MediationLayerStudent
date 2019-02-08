// Author: Tucker Haydon

#include <chrono>
#include <thread>

#include "mediation_layer2d.h"

namespace path_planning {
  
  bool MediationLayer2D::Run() {
    /* ALGORITHM
      for every trajectory in proposed_state
         Forward integrate mediation layer dynamics
         Write resulting trajectory to updated_state
    */

    while(true) {
      const std::string& key = "test";
      Trajectory2D trajectory;

      this->proposed_state_->Add(key, trajectory);
      this->updated_state_->Add(key, trajectory);

      this->proposed_state_->Read("test", trajectory);
      this->updated_state_->Write("test", trajectory);

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return true;
  }
}
