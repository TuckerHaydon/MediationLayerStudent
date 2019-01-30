// Author: Tucker Haydon

#ifndef PATH_PLANNING_DIJKSTRA_H
#define PATH_PLANNING_DIJKSTRA_H

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

#endif
