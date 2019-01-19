// Author: Tucker Haydon

#ifndef MEDIATION_LAYER_DIJKSTRA_H
#define MEDIATION_LAYER_DIJKSTRA_H

#include "graph.h"

class Dijkstra {
  private:
    Graph* graph_;

  public:
    Dijkstra(Graph* graph) : graph_(graph) {};
    std::vector<const Node*> Run(const Node* start, const Node* end) const;
};

#endif
