// Author: Tucker Haydon

#ifndef PATHING_DIJKSTRA_H
#define PATHING_DIJKSTRA_H

#include "graph.h"
namespace pathing {
  class Dijkstra {
    private:
      Graph* graph_;
  
    public:
      Dijkstra(Graph* graph) : graph_(graph) {};
      std::vector<const Node*> Run(const Node* start, const Node* end) const;
  };
}

#endif
