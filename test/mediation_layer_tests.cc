// Author: Tucker Haydon

#include <cstdlib>
#include <iostream>
#include <cassert>

#include "state2d.h"
#include "trajectory2d.h"

using namespace path_planning;

void test_State2D() {
  { // Read non-exist returns false
    State2D state;
    Trajectory2D trajectory;
    assert(false == state.Read("", trajectory));
  }

  { // Write non-exist returns false  
    State2D state;
    Trajectory2D trajectory;
    assert(false == state.Write("", trajectory));
  }
}

int main(int argc, char** argv) {
  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
