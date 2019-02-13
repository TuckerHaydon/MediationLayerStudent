// Author: Tucker Haydon

#pragma once

#include <limits>

#include "node.h"

namespace path_planning {
  /* 
   * POD abstraction of a directed edge for a graph. Every directed edge has a
   * single source, sink, and cost.
   */
  struct DirectedEdge {
    const Node source_;
    const Node sink_;
    double cost_;
  
    DirectedEdge(const Node& source = Node(), 
                 const Node& sink = Node(), 
                 double cost = std::numeric_limits<double>::max()) 
      : source_(source), sink_(sink), cost_(cost) {}; 
  };
}
