// Author: Tucker Haydon

#ifndef PATH_PLANNING_A_STAR_H
#define PATH_PLANNING_A_STAR_H

#include "graph.h"

namespace path_planning {
  class AStar {
    private:
      const Graph* graph_;
  
    public:
      AStar(const Graph* graph) : graph_(graph) {};
      std::vector<Node> Run(const Node& start, const Node& end) const;
  };
}

#endif
