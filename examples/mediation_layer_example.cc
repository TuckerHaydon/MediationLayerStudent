// Author: Tucker Haydon

#include <cstdlib>
#include <memory>

#include "mediation_layer2d.h"
#include "state2d.h"

using namespace mediation_layer;

int main(int argc, char** argv) {

  auto proposed_state = std::make_shared<State2D>();
  auto updated_state = std::make_shared<State2D>();

  MediationLayer2D ml(proposed_state, updated_state);
  ml.Run();

  return EXIT_SUCCESS;
}
