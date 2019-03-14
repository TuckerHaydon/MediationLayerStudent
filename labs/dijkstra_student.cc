// Author: Tucker Haydon

#include "dijkstra_student.h"

namespace mediation_layer {

  // Hiding extraneous information
  using Node2DPtr = std::shared_ptr<Node2D>;

  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {

    ///////////////////////////////////////////////////////////////////
    /// EXAMPLE: 
    ///////////////////////////////////////////////////////////////////
  }

  Dijkstra2D::Work Dijkstra2D::Run(
      const Graph2D& graph, 
      const Node2DPtr start, 
      const Node2DPtr end) {

    // Start the timer to measure run time
    Timer timer;
    timer.Start();


    ///////////////////////////////////////////////////////////////////
    /// EXAMPLE: ACCESSING GRAPH DATA
    ///////////////////////////////////////////////////////////////////

    // Access directed edges eminating from a node
    const std::vector<DirectedEdge2D> edges = graph.Edges(start);
    
    // Iterate through the list of edges
    for(const DirectedEdge2D& edge: edges) {
      // edge.
      
    }


    ///////////////////////////////////////////////////////////////////
    /// EXAMPLE: CREATE THE RETURN VALUE STRUCTURE
    ///////////////////////////////////////////////////////////////////
    Work work;

    work.statistics.num_nodes_explored = 0;
    work.statistics.path_length = 0;
    work.statistics.run_time = timer.Stop();

    work.path = {};

    return work;
  }
  
}
