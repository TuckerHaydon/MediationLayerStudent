// Author: Tucker Haydon
#ifndef MEDIATION_LAYER_NODE_H
#define MEDIATION_LAYER_NODE_H

#include <string>

class Node {
  private:
    std::string id_;

  public:
    static Node NULL_NODE;

    Node(const std::string& id = "DEFAULT") : id_(id) {};

    const std::string& Id() const;

    bool operator==(const Node& other);
};

#endif
