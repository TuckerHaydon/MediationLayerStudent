// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "node.h"

namespace mediation_layer {
  /* 
   * Node implementation for 3D floating point data. Implementation requires
   * that floating point data have no more that 4 significant decimals.
   */
  class Node3D : public Node<Eigen::Matrix<double, 3, 1>> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Node3D(const Eigen::Matrix<double, 3, 1>& data = Eigen::Matrix<double, 3, 1>()) 
        : Node<Eigen::Matrix<double, 3, 1>>(
            data, 
            std::bind(&Node3D::operator==, this, std::placeholders::_1), 
            std::bind(&Node3D::Hash, this)
            ) {}

      bool operator==(const Node3D& other) const;

      size_t Hash() const;

  };

  //============================
  //     IMPLEMENTATION
  //============================
  inline bool Node3D::operator==(const Node3D& other) const {
    const double epsilon = 1e-4;
    return this->data_.isApprox(other.data_, epsilon);
  }

  inline size_t Node3D::Hash() const {
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
