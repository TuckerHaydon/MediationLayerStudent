// Author: Tucker Haydon
#ifndef PATHING_NODE_H
#define PATHING_NODE_H

#include <string>

namespace pathing {
  class Node {
    private:
      std::string id_;
  
    public:
      static Node NULL_NODE;
  
      Node(const std::string& id = "DEFAULT") : id_(id) {};
  
      const std::string& Id() const;
  
      bool operator==(const Node& other) const;
  };
}

#endif
