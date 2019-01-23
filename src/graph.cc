// Author: Tucker Haydon
//
#include <algorithm>
#include <sstream>

#include "graph.h"

namespace path_planning {
  std::vector<DirectedEdge> Graph::EMPTY_EDGE_LIST = {};

  Graph::Graph(const OccupancyGrid& occupancy_grid) {

    // Build node grid
    Node node_grid[occupancy_grid.rows_][occupancy_grid.cols_];
    for(size_t row = 0; row < occupancy_grid.rows_; ++row) {
      for(size_t col = 0; col < occupancy_grid.cols_; ++col) {
        std::stringstream ss;
        ss << "(" << row << "," << col << ")";

        node_grid[row][col] = Node(ss.str());
      }
    }

    // Convert node grid to directed edges and add to graph
    std::vector<DirectedEdge> edges;
    for(int row = 0; row < occupancy_grid.rows_; ++row) {
      for(int col = 0; col < occupancy_grid.cols_; ++col) {
				// If current node is unreachable, pass	
				if(occupancy_grid.occupancy_grid_[row][col]) { continue; }

				// Else, create paths from nearby nodes into this one
        if(row - 1 >= 0) { edges.emplace_back(node_grid[row - 1][col], node_grid[row][col], 1.0); }
        if(col - 1 >= 0) { edges.emplace_back(node_grid[row][col - 1], node_grid[row][col], 1.0); }

        if(row + 1 < occupancy_grid.rows_) { edges.emplace_back(node_grid[row + 1][col], node_grid[row][col], 1.0); }
        if(col + 1 < occupancy_grid.cols_) { edges.emplace_back(node_grid[row][col + 1], node_grid[row][col], 1.0); }
      }
    }

    this->AddEdges(edges);
  }
  
  const std::vector<DirectedEdge>& Graph::Edges(const Node& node) const {
    try {
      return this->edge_graph_.at(node);
    } catch(const std::out_of_range& e) {
      return Graph::EMPTY_EDGE_LIST;
    }
  }
  
  bool Graph::AddEdge(const DirectedEdge& edge) {
    this->edge_graph_[edge.source_].push_back(edge);   
    return true;
  }
  
  bool Graph::AddEdges(const std::vector<DirectedEdge>& edges) {
    std::for_each(edges.begin(), edges.end(), 
        [this](const DirectedEdge& edge){ 
        this->AddEdge(edge); 
    });
    return true;
  }
}
