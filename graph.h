// Author: Tucker Haydon

#ifndef MEDIATION_LAYER_GRAPH_H
#define MEDIATION_LAYER_GRAPH_H

#include <map>
#include <vector>

#include "directed_edge.h"
#include "node.h"

class Graph {
  private: 
    std::map<Node*, std::vector<DirectedEdge*>> edge_graph_;
    static std::vector<DirectedEdge*> EMPTY_EDGE_LIST;

  public:
    Graph(){};
    const std::vector<DirectedEdge*>& GetEdges(Node* node) const;

    bool AddEdge(DirectedEdge* edge);
    bool AddEdges(const std::vector<DirectedEdge*>& edges);
};

#endif
