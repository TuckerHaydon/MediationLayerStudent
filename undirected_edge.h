// Author: Tucker Haydon

#ifndef PATH_PLANNING_UNDIRECTED_EDGE_H
#define PATH_PLANNING_UNDIRECTED_EDGE_H

#include "node.h"

namespace pathing {
  class UndirectedEdge {
    private:
      Node* nodes_[2];
      double weight_;
  
    public:
      UndirectedEdge(Node* a, Node* b) : nodes_{a, b} {};
      UndirectedEdge();
  };
}

#endif
