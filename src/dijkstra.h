// Author: Tucker Haydon

#ifndef PATHING_DIJKSTRA_H
#define PATHING_DIJKSTRA_H

#include "graph.h"

namespace pathing {
  class Dijkstra {
    private:
      const Graph* graph_;
  
    public:
      Dijkstra(const Graph* graph) : graph_(graph) {};
      std::vector<Node> Run(const Node& start, const Node& end) const;
  };
}

#endif
