// Author: Tucker Haydon
#ifndef MEDIATION_LAYER_DIRECTED_EDGE_H
#define MEDIATION_LAYER_DIRECTED_EDGE_H

#include <limits>

#include "node.h"

class DirectedEdge {
  private:
    Node* source_;
    Node* sink_;
    double cost_;

  public:
    DirectedEdge(Node* source = &Node::NULL_NODE, 
                 Node* sink = &Node::NULL_NODE, 
                 double cost = std::numeric_limits<double>::max()) 
      : source_(source), sink_(sink), cost_(cost) {};

    bool SetCost(double cost);
    double Cost() const;

    Node* Source() const;
    Node* Sink() const;

};


#endif
