// Author: Tucker Haydon

#pragma once

#include "graph.h"

namespace path_planning {
  struct AStar {
      AStar(){};
      static std::vector<Node> Run(const Graph& graph, const Node& start, const Node& end);
  };
}
