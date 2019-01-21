// Author: Tucker Haydon

#include "undirected_edge.h"

namespace pathing {
  const std::array<Node, 2>& UndirectedEdge::Nodes() const {
    return this->nodes_;
  }

  const std::array<DirectedEdge, 2>& UndirectedEdge::DirectedEdges() const {
    return this->directed_edges_;
  }
}
