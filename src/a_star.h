// Author: Tucker Haydon

#ifndef PATHING_A_STAR_H
#define PATHING_A_STAR_H

#include "graph.h"

namespace pathing {
  class AStar {
    private:
      const Graph* graph_;
  
    public:
      AStar(const Graph* graph) : graph_(graph) {};
      std::vector<Node> Run(const Node& start, const Node& end) const;
  };
}

#endif
