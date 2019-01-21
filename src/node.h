// Author: Tucker Haydon
#ifndef PATHING_NODE_H
#define PATHING_NODE_H

#include <string>
#include <functional>

namespace pathing {
  class Node {
    private:
      // TODO Change id_ to const
      // TODO Change id_ to custom data type
      std::string id_;
  
    public:
      static Node NULL_NODE;
 
      Node(const std::string& id = "DEFAULT") : id_(id) {};
  
      const std::string& Id() const;
  
      bool operator==(const Node& other) const;

			struct Hash {
			  size_t operator()(const Node& n) const {
					return std::hash<std::string>{}(n.id_);
			  }
			};
  };
}

#endif
