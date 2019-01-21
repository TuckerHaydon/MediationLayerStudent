// Author: Tucker Haydon


#include "occupancy_grid.h"
#include "node.h"
#include "undirected_edge.h"

#include <sstream>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace pathing {

  OccupancyGrid::OccupancyGrid(const std::string& file_path) {
    std::ifstream f(file_path);
    if(!f.is_open()) {
      std::cerr << "File could not be opened." << std::endl;
      std::exit(1);
    }

    f >> this->rows_;
    f >> this->cols_;

    // Allocate memory on the heap for the file
    this->occupancy_grid_ = (bool**)malloc(this->rows_ * sizeof(bool*)); 
    for(size_t idx = 0; idx < this->rows_; ++idx) {
      this->occupancy_grid_[idx] = 
        (bool*) std::malloc(this->cols_ * sizeof(bool));
    }

    // Read in file
    for(size_t row = 0; row < this->rows_; ++row) {
      for(size_t col = 0; col < this->cols_; ++col) {
        f >> this->occupancy_grid_[row][col];
      }
    }

    f.close();

    this->heap_allocated_ = true;
  }

  OccupancyGrid::~OccupancyGrid() {
    if(!this->heap_allocated_) { return; }

    // Deallocate memory on heap
    for(size_t idx = 0; idx < this->rows_; ++idx) {
      std::free(this->occupancy_grid_[idx]);
    }
    std::free(this->occupancy_grid_);
  }

  Graph OccupancyGrid::ToGraph() {
    Graph graph;

    // Build node grid
    Node node_grid[this->rows_][this->cols_];
    for(size_t row = 0; row < this->rows_; ++row) {
      for(size_t col = 0; col < this->cols_; ++col) {
        std::stringstream ss;
        ss << "(" << row << "," << col << ")";

        node_grid[row][col] = Node(ss.str());
      }
    }

    // Convert node grid to directed edges and add to graph
    std::vector<DirectedEdge> edges;
    for(int row = 0; row < this->rows_; ++row) {
      for(int col = 0; col < this->cols_; ++col) {
				// If current node is unreachable, pass	
				if(this->occupancy_grid_[row][col]) { continue; }

				// Else, create paths from nearby nodes into this one
        if(row - 1 >= 0) { edges.emplace_back(node_grid[row - 1][col], node_grid[row][col], 1.0); }
        if(col - 1 >= 0) { edges.emplace_back(node_grid[row][col - 1], node_grid[row][col], 1.0); }

        if(row + 1 < this->rows_) { edges.emplace_back(node_grid[row + 1][col], node_grid[row][col], 1.0); }
        if(col + 1 < this->cols_) { edges.emplace_back(node_grid[row][col + 1], node_grid[row][col], 1.0); }
      }
    }

    graph.AddEdges(edges);

    return graph;
  }

}
