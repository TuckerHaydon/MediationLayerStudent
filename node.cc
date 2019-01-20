// Author: Tucker Haydon

#include "node.h"

namespace pathing {
  Node Node::NULL_NODE("NULL");
  
  bool Node::operator==(const Node& other) const {
    return this->id_ == other.id_;
  }

  bool Node::operator<(const Node& other) const {
    return this->id_ < other.id_;
  }
  
  const std::string& Node::Id() const {
    return this->id_;
  }
}
