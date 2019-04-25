// Author: Tucker Haydon

#pragma once

#include <limits>
#include <memory>
#include <Eigen/Dense>

#include "node_eigen.h"

namespace game_engine {
  // Directed edge for a graph. Every directed edge has a single source, sink,
  // and cost.
  //
  // Directed edge is templated and may contain different types of data.
  // Convenient aliases for 2D and 3D Eigen data are defined at the bottom of
  // this file.
  template <class T>
  struct DirectedEdge {
    // Source pointer
    const std::shared_ptr<T> source_;

    // Sink pointer
    const std::shared_ptr<T> sink_;

    // Cost of the edge
    const double cost_;
  
    // Constructor
    DirectedEdge(const std::shared_ptr<T>& source = nullptr, 
                 const std::shared_ptr<T>& sink = nullptr, 
                 double cost = std::numeric_limits<double>::max()) 
      : source_(source), sink_(sink), cost_(cost) {}; 

    // Getters
    const std::shared_ptr<T>& Source() const;
    const std::shared_ptr<T>& Sink() const;
    const double Cost() const;
  };

  //============================
  //     IMPLEMENTATION
  //============================
  template <class T>
  inline const std::shared_ptr<T>& DirectedEdge<T>::Source() const {
    return this->source_;
  }

  template <class T>
  inline const std::shared_ptr<T>& DirectedEdge<T>::Sink() const {
    return this->sink_;
  }

  template <class T>
  inline const double DirectedEdge<T>::Cost() const {
    return this->cost_;
  }

  using DirectedEdge2D = DirectedEdge<Node2D>;
  using DirectedEdge3D = DirectedEdge<Node3D>;
}
