// Author: Tucker Haydon

#include <stack>

#include "depth_first_search2d.h"

namespace mediation_layer {
  // Hiding extraneous information
  // Do not need to modify this
  using Node2DPtr = std::shared_ptr<Node2D>;

  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct. Functions as a linked list with data. The linked list
    // represents a path. Data contained includes a node and a cost to reach
    // that node. 
    struct NodeVisitor {
      std::shared_ptr<struct NodeVisitor> parent;
      Node2DPtr node_ptr;
      double cost;

      // Equality operator
      bool operator==(const NodeVisitor& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };
  }

  PathInfo DepthFirstSearch2D::Run(
      const Graph2D& graph, 
      const Node2DPtr start_ptr, 
      const Node2DPtr end_ptr) {
    using NodeVisitorPtr = std::shared_ptr<NodeVisitor>;

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();

    // Use these data structures
    std::stack<NodeVisitorPtr> to_explore;
    std::vector<NodeVisitorPtr> explored;

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    // SOME EXAMPLE CODE INCLUDED BELOW
    ///////////////////////////////////////////////////////////////////

    // Create a NodeVisitorPtr
    NodeVisitorPtr nv_ptr = std::make_shared<NodeVisitor>();
    nv_ptr->parent = nullptr;
    nv_ptr->node_ptr = start_ptr;
    nv_ptr->cost = 0;
    to_explore.push(nv_ptr);

    // Create a PathInfo
    PathInfo path_info;
    path_info.details.num_nodes_explored = 0;
    path_info.details.path_length = 0;
    path_info.details.path_cost = 0;
    path_info.details.run_time = timer.Stop();
    path_info.path = {};

    // You must return a PathInfo
    return path_info;
  }
  
}
