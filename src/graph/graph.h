// Author: Tucker Haydon

#pragma once

#include <unordered_map>
#include <vector>

#include "directed_edge.h"
#include "node_eigen.h"

namespace mediation_layer{
  // A graph maintains a set of directed edges connecting various nodes. 
  //
  // Graph is templated and may contain different types of data.  Convenient
  // aliases for 2D and 3D Eigen data are defined at the bottom of this file.
  template <class T>
  class Graph {
    private: 
      // Graph is an abstraction of an unordered map (hash map). Nodes are
      // mapped to a list of DirectedEdges that are eminating from that node.
      //
      // Note that nodes are passed around as shared_ptr while directed edges
      // are copied. Directed edges are lightweight: they only contains pointers
      // and a cost while nodes could contain an arbitrary amount of data
      std::unordered_map<
        std::shared_ptr<T>, 
        std::vector<DirectedEdge<T>>,
        class T::HashPointer,
        class T::EqualsPointer> edge_graph_;
  
    public:
      // Constructor
      Graph(const std::vector<DirectedEdge<T>>& edges = {});
  
      // Add an edge to the graph
      void AddEdge(const DirectedEdge<T>& edge);

      // Add a vector of edges to the graph
      void AddEdges(const std::vector<DirectedEdge<T>>& edges);

      // Returns a vector of edges that are eminating from a given node
      const std::vector<DirectedEdge<T>> Edges(const std::shared_ptr<T>& node) const;

      // Returns a list of nodes that are neighbors of a given node. A neighbor
      // node is defined as a sink node for an edge whose source node is the
      // given node
      const std::vector<std::shared_ptr<T>> Neighbors(const std::shared_ptr<T>& node) const;

      // Returns the size of the graph. The size is the number of directed edges
      // in the graph
      size_t Size() const;

  };

  //============================
  //     IMPLEMENTATION
  //============================
  template <class T>
  inline Graph<T>::Graph(const std::vector<DirectedEdge<T>>& edges) {
    this->AddEdges(edges);
  }
  
  template <class T>
  inline void Graph<T>::AddEdge(const DirectedEdge<T>& edge) {
    this->edge_graph_[edge.source_].push_back(edge);   
  }
  
  template <class T>
  inline void Graph<T>::AddEdges(const std::vector<DirectedEdge<T>>& edges) {
    std::for_each(edges.begin(), edges.end(), 
        [this](const DirectedEdge<T>& edge){ 
        this->AddEdge(edge); 
    });
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
  inline const std::vector<std::shared_ptr<T>> Graph<T>::Neighbors(const std::shared_ptr<T>& node) const {
    const std::vector<DirectedEdge<T>>& edges = this->Edges(node);
    std::vector<std::shared_ptr<T>> neighbors;
    neighbors.reserve(edges.size());
    for(const DirectedEdge<T>& edge: edges){
      neighbors.push_back(edge.Sink());
    }
    return neighbors;
  }

  template <class T>
  inline size_t Graph<T>::Size() const {
    return this->edge_graph_.size();
  }

  using Graph2D = Graph<Node2D>;
  using Graph3D = Graph<Node3D>;
}
