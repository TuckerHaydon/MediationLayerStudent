// Author: Tucker Haydon

#include "dijkstra2d.h"

namespace mediation_layer {
  // Hiding extraneous information
  // Do not need to modify this
  using Node2DPtr = std::shared_ptr<Node2D>;

  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {

  }

  Dijkstra2D::Work Dijkstra2D::Run(
      const Graph2D& graph, 
      const Node2DPtr start_node, 
      const Node2DPtr end_node) {

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();


    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    ///////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////
    // CREATE THE RETURN VALUE STRUCTURE
    // MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Work work;

    work.statistics.num_nodes_explored = 1;
    work.statistics.path_length = 1;
    work.statistics.run_time = timer.Stop();

    work.path.push_back(start_node);

    return work;
  }
  
}
