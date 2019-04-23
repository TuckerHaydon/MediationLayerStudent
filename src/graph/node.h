// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <type_traits>
#include <memory>

namespace mediation_layer {
  // Interface defining a Node data for use in DirectedEdge and Graph data
  // structures. Nodes contain data, but must also implement equals and hash
  // functions for storage in STL containers.
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

      // Equality operator. Nodes are equal if the their equality functions
      // evaluate to true
      bool operator==(const Node& other) const {
        return this->f_equals_(other.data_);
      }

      // Equals structure. Convience structure for STL containers
      struct Equals {
        bool operator()(const Node& lhs, const Node& rhs) const {
          return lhs == rhs;
        }
      };

      // Equals structure for shared pointers. Convenience structure for STL
      // containers
      struct EqualsPointer {  
        bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
          return *lhs == *rhs;
        }
      };

      // Hash structure. Convenience structure for STL containers
      struct Hash {
        size_t operator()(const Node& node) const {
          return node.f_hash_();
        }
      };

      // Hash structure for shared pointers. Convenience structure for STL
      // containers
      struct HashPointer {
        size_t operator()(const std::shared_ptr<Node>& node_ptr) const {
          return node_ptr->f_hash_();
        }
      };
  };
}
