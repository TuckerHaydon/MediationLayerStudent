// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "node.h"

namespace game_engine {
  // Abstract node implementation that contains Eigen data. Due to constraints
  // on hash function precision, users of NodeEigen should not expect their data
  // to contain more than 3 decimal points of accuracy. See the hash function
  // documentation below as to why.
  //
  // NodeEigen is templated and may contain different sizes of data.  Convenient
  // aliases for 2D and 3D Eigen data are defined at the bottom of this file.
  template <int D>
  class NodeEigen : public Node<Eigen::Matrix<double, D, 1>> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Constructor
      NodeEigen(const Eigen::Matrix<double, D, 1>& data = Eigen::Matrix<double, D, 1>()) 
        : Node<Eigen::Matrix<double, D, 1>>(
            data, 
            std::bind(&NodeEigen::operator==, this, std::placeholders::_1), 
            std::bind(&NodeEigen::Hash, this)
            ) {}

      // Equality operator. NodeEigen objects are equal if their elements are
      // component-wise within epsilon of each other.
      //
      // epsilon = 1e-4
      bool operator==(const NodeEigen& other) const;

      // Hash function. Hash functions for floating point numbers are
      // complicated due to numerical errors. 3.0 != 3.0 always. To solve this,
      // when computing the hash, floating point numbers are multiplied by 10^4
      // and truncated to the nearest integer. Determining the hash of a tuple
      // of integers is easy. Thus, NodeEigen should not be used if the data
      // required more than 3 decimal points of precision.
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
