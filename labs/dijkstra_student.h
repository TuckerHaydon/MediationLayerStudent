// Author: Tucker Haydon

#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include "graph.h"
#include "timer.h"

namespace mediation_layer {

  struct Dijkstra2D {

    // Return value structure
    struct Work {
      struct Statistics {
        size_t num_nodes_explored{0};
        size_t path_length{0};
        std::chrono::duration<double> run_time{0};
 
        void Print() const {
          std::cout << "Dijkstra Statistics" << std::endl;
          std::cout << "\t" << "Number of Nodes Explored: " << num_nodes_explored << std::endl;
          std::cout << "\t" << "Number of Nodes in Path: " << path_length << std::endl;
          std::cout << "\t" << "Run Time: " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(run_time).count() 
            << " ms" << std::endl;
          std::cout << std::endl;
        }
      };
     
      std::vector<std::shared_ptr<Node2D>> path;
      Statistics statistics; 
    };

    // Run dijkstras on a graph.
    Work Run(const Graph2D& graph, const std::shared_ptr<Node2D> start, const std::shared_ptr<Node2D> end);
  };
}
