// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "node.h"

namespace mediation_layer {
  /* 
   * Node implementation for 2D floating point data. Implementation requires
   * that floating point data have no more that 4 significant decimals.
   */
  class Node2D : public Node<Eigen::Vector2d> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Node2D(const Eigen::Vector2d& data = Eigen::Vector2d()) 
        : Node<Eigen::Vector2d>(
            data, 
            std::bind(&Node2D::operator==, this, std::placeholders::_1), 
            std::bind(&Node2D::Hash, this)
            ) {}

      bool operator==(const Node2D& other) const;

      size_t Hash() const;

  };

  //============================
  //     IMPLEMENTATION
  //============================
  inline bool Node2D::operator==(const Node2D& other) const {
    const double epsilon = 1e-4;
    return this->data_.isApprox(other.data_, epsilon);
  }

  inline size_t Node2D::Hash() const {
    const auto RoundDouble = [](const double d) {
      // Multiply by 10,000 and round to the nearest integer
      return static_cast<size_t>(d * 1e4);
    };

    size_t seed = this->data_.size()*sizeof(double);
    for(size_t idx = 0; idx < this->data_.size(); ++idx) {
      seed ^= RoundDouble(this->data_.data()[idx]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;    
  }
}
