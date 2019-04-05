// Author: Tucker Haydon

#pragma once

#include "automation_protocol.h"

namespace mediation_layer {
  class TestAP : public AutomationProtocol3D {
    private:

    public:   
      std::unordered_map<std::string, Trajectory3D> UpdateTrajectories(
          std::shared_ptr<GameSnapshot3D> snapshot,
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names) override;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  std::unordered_map<std::string, Trajectory3D> UpdateTrajectories(
      std::shared_ptr<GameSnapshot3D> snapshot,
      const std::vector<std::string>& friendly_names,
      const std::vector<std::string>& enemy_names) {
    std::unordered_map<std::string, Trajectory3D> m;
    return m;
  }
}
