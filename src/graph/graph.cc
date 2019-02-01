// Author: Tucker Haydon

#include <algorithm>
#include <iostream>

#include "graph.h"

namespace path_planning {
  std::vector<DirectedEdge> Graph::EMPTY_EDGE_LIST = {};

  Graph::Graph(const OccupancyGrid2D& occupancy_grid) {
    std::cout << occupancy_grid.SizeY() << " " << occupancy_grid.SizeX() << std::endl;

    // Build node grid
    Node node_grid[occupancy_grid.SizeY()][occupancy_grid.SizeX()];
    for(size_t row = 0; row < occupancy_grid.SizeY(); ++row) {
      for(size_t col = 0; col < occupancy_grid.SizeX(); ++col) {
        int data[2] = {static_cast<int>(row), static_cast<int>(col)};
        size_t data_size = 2 * sizeof(int);
        node_grid[row][col].SetData(reinterpret_cast<uint8_t*>(&data), data_size);
      }
    }

    // Convert node grid to directed edges and add to graph
    std::vector<DirectedEdge> edges;
    for(int row = 0; row < occupancy_grid.SizeY(); ++row) {
      for(int col = 0; col < occupancy_grid.SizeX(); ++col) {
				// If current node is unreachable, pass	
				if(true == occupancy_grid.IsOccupied(row, col)) { continue; }

				// Else, create paths from nearby nodes into this one
        // 8 node surrounding the current node
        if(row - 1 >= 0) { edges.emplace_back(node_grid[row - 1][col], node_grid[row][col], 1.0); }
        if(col - 1 >= 0) { edges.emplace_back(node_grid[row][col - 1], node_grid[row][col], 1.0); }

        if(row + 1 < occupancy_grid.SizeY()) { edges.emplace_back(node_grid[row + 1][col], node_grid[row][col], 1.0); }
        if(col + 1 < occupancy_grid.SizeX()) { edges.emplace_back(node_grid[row][col + 1], node_grid[row][col], 1.0); }

        if(row - 1 >= 0 && col - 1 >= 0) { 
          edges.emplace_back(node_grid[row - 1][col - 1], node_grid[row][col], 1.0); }
        if(row - 1 >= 0 && col + 1 < occupancy_grid.SizeX()) { 
          edges.emplace_back(node_grid[row - 1][col + 1], node_grid[row][col], 1.0); }
        if(row + 1 < occupancy_grid.SizeY() && col - 1 >= 0) { 
          edges.emplace_back(node_grid[row + 1][col - 1], node_grid[row][col], 1.0); }
        if(row + 1 < occupancy_grid.SizeY() && col +1 < occupancy_grid.SizeX()) { 
          edges.emplace_back(node_grid[row + 1][col + 1], node_grid[row][col], 1.0); }
      }
    }

    this->AddEdges(edges);
  } 

  const std::vector<DirectedEdge>& Graph::Edges(const Node& node) const {
    try {
      return this->edge_graph_.at(node);
    } catch(const std::out_of_range& e) {
      return EMPTY_EDGE_LIST;
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
