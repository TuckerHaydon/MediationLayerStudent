// Author: Tucker Haydon

#pragma once

#include <unordered_map>
#include <vector>

#include "directed_edge.h"
#include "node.h"

namespace path_planning{
  /*
   * Abstraction of a graph. A graph maintains a set of directed edges
   * connecting various nodes.
   */
  template <class T>
  class Graph {
    private: 
      std::unordered_map<
        std::shared_ptr<Node<T>>, 
        std::vector<DirectedEdge<T>>,
        class Node<T>::HashPointer,
        class Node<T>::EqualsPointer> edge_graph_;
  
    public:
      Graph(const std::vector<DirectedEdge<T>>& edges = {});

      const std::vector<DirectedEdge<T>> Edges(const std::shared_ptr<Node<T>>& node) const;
  
      bool AddEdge(const DirectedEdge<T>& edge);
      bool AddEdges(const std::vector<DirectedEdge<T>>& edges);

  };

  //============================
  //     IMPLEMENTATION
  //============================
  template <class T>
  inline Graph<T>::Graph(const std::vector<DirectedEdge<T>>& edges) {
    this->AddEdges(edges);
  }

  template <class T>
  inline const std::vector<DirectedEdge<T>> Graph<T>::Edges(const std::shared_ptr<Node<T>>& node) const {
    try {
      return this->edge_graph_.at(node);
    } catch(const std::out_of_range& e) {
      return {};
    }
  }
  
  template <class T>
  inline bool Graph<T>::AddEdge(const DirectedEdge<T>& edge) {
    this->edge_graph_[edge.source_].push_back(edge);   
    return true;
  }
  
  template <class T>
  inline bool Graph<T>::AddEdges(const std::vector<DirectedEdge<T>>& edges) {
    std::for_each(edges.begin(), edges.end(), 
        [this](const DirectedEdge<T>& edge){ 
        this->AddEdge(edge); 
    });
    return true;
  }

  using Graph2D = Graph<Eigen::Matrix<double, 2, 1>>;
  using Graph3D = Graph<Eigen::Matrix<double, 3, 1>>;

  // inline Graph::Graph(const OccupancyGrid2D& occupancy_grid) {
  //   std::cout << occupancy_grid.SizeY() << " " << occupancy_grid.SizeX() << std::endl;

  //   // Build node grid
  //   Node node_grid[occupancy_grid.SizeY()][occupancy_grid.SizeX()];
  //   for(size_t row = 0; row < occupancy_grid.SizeY(); ++row) {
  //     for(size_t col = 0; col < occupancy_grid.SizeX(); ++col) {
  //       int data[2] = {static_cast<int>(row), static_cast<int>(col)};
  //       size_t data_size = 2 * sizeof(int);
  //       node_grid[row][col].SetData(reinterpret_cast<uint8_t*>(&data), data_size);
  //     }
  //   }

  //   // Convert node grid to directed edges and add to graph
  //   std::vector<DirectedEdge> edges;
  //   for(int row = 0; row < occupancy_grid.SizeY(); ++row) {
  //     for(int col = 0; col < occupancy_grid.SizeX(); ++col) {
	// 			// If current node is unreachable, pass	
	// 			if(true == occupancy_grid.IsOccupied(row, col)) { continue; }

  //       constexpr double ADJACENT_COST = 1.0;
  //       constexpr double DIAGONAL_COST = std::sqrt(2);
	// 			// Else, create paths from nearby nodes into this one
  //       // 8 node surrounding the current node
  //       if(row - 1 >= 0) { edges.emplace_back(node_grid[row - 1][col], node_grid[row][col], ADJACENT_COST); }
  //       if(col - 1 >= 0) { edges.emplace_back(node_grid[row][col - 1], node_grid[row][col], ADJACENT_COST); }

  //       if(row + 1 < occupancy_grid.SizeY()) { edges.emplace_back(node_grid[row + 1][col], node_grid[row][col], ADJACENT_COST); }
  //       if(col + 1 < occupancy_grid.SizeX()) { edges.emplace_back(node_grid[row][col + 1], node_grid[row][col], ADJACENT_COST); }

  //       if(row - 1 >= 0 && col - 1 >= 0) { 
  //         edges.emplace_back(node_grid[row - 1][col - 1], node_grid[row][col], DIAGONAL_COST); }
  //       if(row - 1 >= 0 && col + 1 < occupancy_grid.SizeX()) { 
  //         edges.emplace_back(node_grid[row - 1][col + 1], node_grid[row][col], DIAGONAL_COST); }
  //       if(row + 1 < occupancy_grid.SizeY() && col - 1 >= 0) { 
  //         edges.emplace_back(node_grid[row + 1][col - 1], node_grid[row][col], DIAGONAL_COST); }
  //       if(row + 1 < occupancy_grid.SizeY() && col +1 < occupancy_grid.SizeX()) { 
  //         edges.emplace_back(node_grid[row + 1][col + 1], node_grid[row][col], DIAGONAL_COST); }
  //     }
  //   }

  //   this->AddEdges(edges);
  // } 
}
