// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <type_traits>
#include <memory>

namespace path_planning {
  template <class T>
  class Node {
    protected:
      const T data_;
      const std::function<bool(const T& other)> f_equals_;
      const std::function<size_t()> f_hash_;

    public:
      Node(const T& data = T(),
           std::function<bool(const T&)> f_equals = [](const T& t){ return false; }, 
           std::function<size_t()> f_hash = [](const T& t) { return 0; })
        : data_(data),
          f_equals_(f_equals),
          f_hash_(f_hash) {}

      const T& Data() const {
        return data_;
      }

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
