// Author: Tucker Haydon

#include "undirected_edge.h"

namespace pathing {
  const std::array<const Node, 2>& Nodes() const {
    return this->nodes_;
  }

  const std::array<const DirectedEdge, 2>& DirectedEdges() const {
    return this->directed_edges_;
  }
}
