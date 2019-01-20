// Author: Tucker Haydon
//
#include <algorithm>

#include "graph.h"

namespace pathing {
  std::vector<const DirectedEdge*> Graph::EMPTY_EDGE_LIST = {};
  
  const std::vector<const DirectedEdge*>& Graph::GetEdges(const Node* node) const {
    try {
      return this->edge_graph_.at(node);
    } catch(const std::out_of_range& e) {
      return Graph::EMPTY_EDGE_LIST;
    }
  }
  
  bool Graph::AddEdge(const DirectedEdge* edge) {
    this->edge_graph_[edge->Source()].push_back(edge);   
    return true;
  }
  
  bool Graph::AddEdges(const std::vector<const DirectedEdge*>& edges) {
    std::for_each(edges.begin(), edges.end(), 
        [this](const DirectedEdge* edge){ this->AddEdge(edge); });
    return true;
  }
}
