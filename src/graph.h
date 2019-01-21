// Author: Tucker Haydon

#ifndef PATHING_GRAPH_H
#define PATHING_GRAPH_H

#include <unordered_map>
#include <vector>

#include "directed_edge.h"
#include "undirected_edge.h"
#include "node.h"

namespace pathing{
  class Graph {
    private: 
      std::unordered_map<Node, std::vector<DirectedEdge>, Node::Hash> edge_graph_;
      static std::vector<DirectedEdge> EMPTY_EDGE_LIST;
  
    public:
      Graph(){};
      const std::vector<DirectedEdge>& Edges(const Node& node) const;
  
      bool AddEdge(const DirectedEdge& edge);
      bool AddEdges(const std::vector<DirectedEdge>& edges);

      bool AddEdge(const UndirectedEdge& edge);
      bool AddEdges(const std::vector<UndirectedEdge>& edges);
  };
}

#endif
