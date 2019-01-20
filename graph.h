// Author: Tucker Haydon

#ifndef PATH_PLANNING_GRAPH_H
#define PATH_PLANNING_GRAPH_H

#include <map>
#include <vector>

#include "directed_edge.h"
#include "node.h"

namespace pathing{
  class Graph {
    private: 
      std::map<const Node*, std::vector<const DirectedEdge*>> edge_graph_;
      static std::vector<const DirectedEdge*> EMPTY_EDGE_LIST;
  
    public:
      Graph(){};
      const std::vector<const DirectedEdge*>& GetEdges(const Node* node) const;
  
      bool AddEdge(const DirectedEdge* edge);
      bool AddEdges(const std::vector<const DirectedEdge*>& edges);
  };
}

#endif
