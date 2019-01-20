// Author: Tucker Haydo

#include "directed_edge.h"

namespace pathing {
  double DirectedEdge::Cost() const {
    return this->cost_;
  }
  
  const Node* DirectedEdge::Source() const {
    return this->source_;
  }
  
  const Node* DirectedEdge::Sink() const {
    return this->sink_;
  }
}
