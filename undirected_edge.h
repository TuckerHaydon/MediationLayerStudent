// Author: Tucker Haydon

#ifndef MEDIATION_LAYER_UNDIRECTED_EDGE_H
#define MEDIATION_LAYER_UNDIRECTED_EDGE_H

#include "node.h"

// Definition
class UndirectedEdge {
  private:
    Node* nodes_[2];
    double weight_;

  public:
    UndirectedEdge(Node* a, Node* b) : nodes_{a, b} {};
    UndirectedEdge();
};

#endif
