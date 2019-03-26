// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <type_traits>
#include <memory>

namespace mediation_layer {

  // Abstract node class for use in Graph and DirectedEdge. A node is a simple
  // data container.
  template <class T>
  class Node {
    protected:
      // Data
      const T data_;

      // Required equality function so that the node may be used in std
      // containers
      const std::function<bool(const T& other)> f_equals_;

      // Required hash function so that the node may be used in std containers
      const std::function<size_t()> f_hash_;

    public:
      // Constructor
      Node(const T& data = T(),
           std::function<bool(const T&)> f_equals = [](const T& t){ return false; }, 
           std::function<size_t()> f_hash = [](const T& t) { return 0; })
        : data_(data),
          f_equals_(f_equals),
          f_hash_(f_hash) {}

      // Getter
      const T& Data() const {
        return data_;
      }

      // Operators for std containers
      bool operator==(const Node& other) const {
        return this->f_equals_(other.data_);
      }

      struct Equals {
        bool operator()(const Node& lhs, const Node& rhs) const {
          return lhs == rhs;
        }
      };

      struct EqualsPointer {  
        bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
          return *lhs == *rhs;
        }
      };

      struct Hash {
        size_t operator()(const Node& node) const {
          return node.f_hash_();
        }
      };

      struct HashPointer {
        size_t operator()(const std::shared_ptr<Node>& node_ptr) const {
          return node_ptr->f_hash_();
        }
      };
  };
}
