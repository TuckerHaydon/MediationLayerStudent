// Author: Tucker Haydon

#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include "graph.h"
#include "timer.h"
#include "path_info.h"

namespace game_engine {

  struct DepthFirstSearch2D {
    using Node2DPtr = std::shared_ptr<Node2D>;

    // Students will implement this function
    PathInfo Run(const Graph2D& graph, const Node2DPtr start_ptr, const Node2DPtr end_ptr);
  };
}
