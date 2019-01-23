// Author: Tucker Haydon

#ifndef PATH_PLANNING_NODE_H
#define PATH_PLANNING_NODE_H

#include <string>
#include <functional>

namespace path_planning {
  /*
   * POD abstraction of a node in a graph. Each node contains information
   * represented by a string.
   */
  struct Node {
      // TODO Change id_ to custom data type
      std::string id_;
  
      static Node NULL_NODE;

      Node(const std::string& id = "DEFAULT") : id_(id) {};
  
      bool operator==(const Node& other) const {
        return this->id_ == other.id_;
      };

			struct Hash {
			  size_t operator()(const Node& n) const {
					return std::hash<std::string>{}(n.id_);
			  }
			};
  };
}

#endif
