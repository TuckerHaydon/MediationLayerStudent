// Author: Tucker Haydon

#include "dijkstra2d.h"

namespace mediation_layer {
  // Hiding extraneous information
  // Do not need to modify this
  using Node2DPtr = std::shared_ptr<Node2D>;

  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {

  }

  PathInfo Dijkstra2D::Run(
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
    PathInfo path_info;

    path_info.details.num_nodes_explored = 1;
    path_info.details.path_length = 1;
    path_info.details.run_time = timer.Stop();

    path_info.path.push_back(start_node);

    return path_info;
  }
  
}
