// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "node.h"

namespace mediation_layer {
  // Abstract node that contains Eigen data
  template <int D>
  class NodeEigen : public Node<Eigen::Matrix<double, D, 1>> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      NodeEigen(const Eigen::Matrix<double, D, 1>& data = Eigen::Matrix<double, D, 1>()) 
        : Node<Eigen::Matrix<double, D, 1>>(
            data, 
            std::bind(&NodeEigen::operator==, this, std::placeholders::_1), 
            std::bind(&NodeEigen::Hash, this)
            ) {}

      bool operator==(const NodeEigen& other) const;

      size_t Hash() const;

  };

  //============================
  //     IMPLEMENTATION
  //============================
  template <int D>
  inline bool NodeEigen<D>::operator==(const NodeEigen<D>& other) const {
    const double epsilon = 1e-4;
    return this->data_.isApprox(other.data_, epsilon);
  }

  template <int D>
  inline size_t NodeEigen<D>::Hash() const {
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

  using Node2D = NodeEigen<2>;
  using Node3D = NodeEigen<3>;
}
