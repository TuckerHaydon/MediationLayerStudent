// Author: Tucker Haydon

#pragma once

#include <unordered_map>
#include <vector>

#include "directed_edge.h"
#include "node2d.h"
#include "node3d.h"

namespace mediation_layer{
  /*
   * Abstraction of a graph. A graph maintains a set of directed edges
   * connecting various nodes.
   */
  template <class T>
  class Graph {
    private: 
      std::unordered_map<
        std::shared_ptr<T>, 
        std::vector<DirectedEdge<T>>,
        class T::HashPointer,
        class T::EqualsPointer> edge_graph_;
  
    public:
      Graph(const std::vector<DirectedEdge<T>>& edges = {});

      const std::vector<DirectedEdge<T>> Edges(const std::shared_ptr<T>& node) const;
  
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
  inline const std::vector<DirectedEdge<T>> Graph<T>::Edges(const std::shared_ptr<T>& node) const {
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

  using Graph2D = Graph<Node2D>;
  using Graph3D = Graph<Node3D>;
}
