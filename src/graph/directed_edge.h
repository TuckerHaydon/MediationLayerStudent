// Author: Tucker Haydon

#pragma once

#include <limits>
#include <memory>
#include <Eigen/Dense>

#include "node.h"

namespace mediation_layer {
  /* 
   * POD abstraction of a directed edge for a graph. Every directed edge has a
   * single source, sink, and cost.
   */
  template <class T>
  struct DirectedEdge {
    const std::shared_ptr<Node<T>> source_;
    const std::shared_ptr<Node<T>> sink_;
    double cost_;
  
    DirectedEdge(const std::shared_ptr<Node<T>>& source = nullptr, 
                 const std::shared_ptr<Node<T>>& sink = nullptr, 
                 double cost = std::numeric_limits<double>::max()) 
      : source_(source), sink_(sink), cost_(cost) {}; 

    const std::shared_ptr<Node<T>>& Source() const;
    const std::shared_ptr<Node<T>>& Sink() const;
  };

  //============================
  //     IMPLEMENTATION
  //============================
  template <class T>
  inline const std::shared_ptr<Node<T>>& DirectedEdge<T>::Source() const {
    return this->source_;
  }

  template <class T>
  inline const std::shared_ptr<Node<T>>& DirectedEdge<T>::Sink() const {
    return this->sink_;
  }

  using DirectedEdge2D = DirectedEdge<Eigen::Matrix<double, 2, 1>>;
  using DirectedEdge3D = DirectedEdge<Eigen::Matrix<double, 3, 1>>;
}
