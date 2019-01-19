// Author: Tucker Haydon

#include "node.h"

Node Node::NULL_NODE("NULL");

bool Node::operator==(const Node& other) {
  return this->id_ == other.id_;
}

const std::string& Node::Id() const {
  return this->id_;
}
