// Author: Tucker Haydon

#pragma once

#include "graph.h"

namespace path_planning {
  /*
   * Implementation of Dijkstra's path-finding algorithm.
   */
  struct Dijkstra {
      Dijkstra(){};
      static std::vector<Node> Run(const Graph& graph, 
                                   const Node& start, 
                                   const Node& end);
  };
}
