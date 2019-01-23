// Author: Tucker Haydon

#ifndef PATH_PLANNING_A_STAR_H
#define PATH_PLANNING_A_STAR_H

#include "graph.h"

namespace path_planning {
  struct AStar {
      AStar(){};
      static std::vector<Node> Run(const Graph& graph, const Node& start, const Node& end);
  };
}

#endif
