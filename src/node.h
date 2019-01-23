// Author: Tucker Haydon

#ifndef PATH_PLANNING_NODE_H
#define PATH_PLANNING_NODE_H

#include <string>
#include <functional>

namespace path_planning {
  struct Node {
      Node(const std::string& id = "DEFAULT") : id_(id) {};

      // TODO Change id_ to const
      // TODO Change id_ to custom data type
      std::string id_;
  
      static Node NULL_NODE;
  
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
