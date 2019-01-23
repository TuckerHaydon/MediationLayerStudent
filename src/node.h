// Author: Tucker Haydon

#ifndef PATH_PLANNING_NODE_H
#define PATH_PLANNING_NODE_H

#include <functional>
#include <array>
#include <string>
#include <iostream>

namespace path_planning {
  /*
   * POD abstraction of a node in a graph. Each node contains information
   * represented by a string.
   */
  struct Node {
    // TODO Change id_ to custom data type
    std::array<int64_t, 2> id_;
  
    static Node NULL_NODE;

    Node(const std::array<int64_t, 2> id = {0, 0}) : id_{id} {};

    friend std::ostream& operator<<(std::ostream &os, const Node &n) {
      os << std::to_string(n.id_[0]) << ", " << std::to_string(n.id_[1]);
      return os;
    }
  
    bool operator==(const Node& other) const {
      return 
        this->id_[0] == other.id_[0] &&
        this->id_[1] == other.id_[1];
    };

		struct Hash {
		  size_t operator()(const Node& n) const {
        return
          31 * std::hash<int64_t>{}(n.id_[0]) + 
          53 * std::hash<int64_t>{}(n.id_[1]);
		  }
		};
  };
}

#endif
