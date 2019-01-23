// Author: Tucker Haydon

#ifndef PATH_PLANNING_DIRECTED_EDGE_H
#define PATH_PLANNING_DIRECTED_EDGE_H

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
  
    DirectedEdge(const Node& source = Node::NULL_NODE, 
                 const Node& sink = Node::NULL_NODE, 
                 double cost = std::numeric_limits<double>::max()) 
      : source_(source), sink_(sink), cost_(cost) {}; 
  };
}

#endif
