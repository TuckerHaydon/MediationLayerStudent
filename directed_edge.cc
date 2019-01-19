// Author: Tucker Haydo

#include "directed_edge.h"

bool DirectedEdge::SetCost(double cost) {
  this->cost_ = cost;
  return true;
}

double DirectedEdge::Cost() const {
  return this->cost_;
}

Node* DirectedEdge::Source() const {
  return this->source_;
}

Node* DirectedEdge::Sink() const {
  return this->sink_;
}
