// Author: Tucker Haydon

#include "undirected_edge.h"

namespace pathing {
  UndirectedEdge::UndirectedEdge() 
    : UndirectedEdge(&Node::NULL_NODE, &Node::NULL_NODE) {};
}
