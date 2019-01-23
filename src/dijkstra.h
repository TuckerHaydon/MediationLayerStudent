// Author: Tucker Haydon

#ifndef PATH_PLANNING_DIJKSTRA_H
#define PATH_PLANNING_DIJKSTRA_H

#include "graph.h"

namespace path_planning {
  class Dijkstra {
    private:
      const Graph* graph_;
  
    public:
      Dijkstra(const Graph* graph) : graph_(graph) {};
      std::vector<Node> Run(const Node& start, const Node& end) const;
  };
}

#endif
