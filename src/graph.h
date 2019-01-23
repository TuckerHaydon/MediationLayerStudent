// Author: Tucker Haydon

#ifndef PATH_PLANNING_GRAPH_H
#define PATH_PLANNING_GRAPH_H

#include <unordered_map>
#include <vector>

#include "directed_edge.h"
#include "node.h"
#include "occupancy_grid.h"

namespace path_planning{
  class Graph {
    private: 
      std::unordered_map<Node, std::vector<DirectedEdge>, Node::Hash> edge_graph_;
      static std::vector<DirectedEdge> EMPTY_EDGE_LIST;
  
    public:
      Graph(){};
      Graph(const OccupancyGrid& occupancy_grid);

      const std::vector<DirectedEdge>& Edges(const Node& node) const;
  
      bool AddEdge(const DirectedEdge& edge);
      bool AddEdges(const std::vector<DirectedEdge>& edges);
  };
}

#endif
