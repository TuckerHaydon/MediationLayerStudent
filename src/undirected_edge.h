// Author: Tucker Haydon

#ifndef PATHING_UNDIRECTED_EDGE_H
#define PATHING_UNDIRECTED_EDGE_H

#include <array>
#include <limits>

#include "node.h"
#include "directed_edge.h"

namespace pathing {
  class UndirectedEdge {
    private:
      const std::array<Node, 2> nodes_;
      const std::array<DirectedEdge, 2> directed_edges_;
      double cost_;
  
    public:
      UndirectedEdge(const Node& a = Node::NULL_NODE, 
                     const Node& b = Node::NULL_NODE,
                     double cost = std::numeric_limits<double>::max()) 
        : nodes_({{a, b}}),
          directed_edges_({{DirectedEdge(a, b, cost), DirectedEdge(b, a, cost)}}),
          cost_(cost)
        {};

      const std::array<Node, 2>& Nodes() const;
      const std::array<DirectedEdge, 2>& DirectedEdges() const;
  };
}

#endif
