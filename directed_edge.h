// Author: Tucker Haydon
#ifndef PATHING_DIRECTED_EDGE_H
#define PATHING_DIRECTED_EDGE_H

#include <limits>

#include "node.h"

namespace pathing {
  class DirectedEdge {
    private:
      const Node* source_;
      const Node* sink_;
      double cost_;
  
    public:
      DirectedEdge(const Node* source = &Node::NULL_NODE, 
                   const Node* sink = &Node::NULL_NODE, 
                   double cost = std::numeric_limits<double>::max()) 
        : source_(source), sink_(sink), cost_(cost) {};
  
      double Cost() const;
      const Node* Source() const;
      const Node* Sink() const;
  
  };
}

#endif
