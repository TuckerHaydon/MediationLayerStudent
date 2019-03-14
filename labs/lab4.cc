// Author: PUT YOUR NAME HERE

#include <cstdlib>
#include <iostream>

#include "dijkstra2d.h"
#include "occupancy_grid2d.h"

using namespace mediation_layer;
using Node2DPtr = std::shared_ptr<Node2D>;


///////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void RunDepthFirstSearch(
    const Graph2D& graph,
    const Node2DPtr& start_node,
    const Node2DPtr& end_node);

void RunDijkstra(
    const Graph2D& graph,
    const Node2DPtr& start_node,
    const Node2DPtr& end_node);

void RunAStar(
    const Graph2D& graph,
    const Node2DPtr& start_node,
    const Node2DPtr& end_node);

///////////////////////////////////////////////////////////////////
// MAIN FUNCTION
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  if(argc != 2) {
    std::cerr << "Usage: ./lab4 occupancy_grid_file" << std::endl;
    return EXIT_FAILURE;
  }

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(argv[1]);

  // Transform an occupancy grid into a graph
  Graph2D graph = occupancy_grid.AsGraph();

  // Define the start and end points
  Node2DPtr start_node = std::make_shared<Node2D>(Eigen::Vector2d(0,0));
  Node2DPtr end_node = std::make_shared<Node2D>(Eigen::Vector2d(4,4));

  // Run the path planning algorithms
  RunDepthFirstSearch(graph, start_node, end_node);
  RunDijkstra(graph, start_node, end_node);
  RunAStar(graph, start_node, end_node);

  return EXIT_SUCCESS;
}

///////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void RunDepthFirstSearch(
    const Graph2D& graph,
    const Node2DPtr& start_node,
    const Node2DPtr& end_node) {
  
  std::cout << "============================================" << std::endl;
  std::cout << "======== RUNNING DEPTH FIRST SEARCH ========" << std::endl;
  std::cout << "============================================" << std::endl;

  // Run Dijkstra
  // Dijkstra2D dijkstra;
  // Dijkstra2D::Work work = dijkstra.Run(graph, start_node, end_node);

  // Print the solution
  // work.statistics.Print();

  // std::cout << "===== PATH =====" << std::endl;
  // for(const Node2DPtr& node: work.path) {
  //   std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  // }

  // std::cout << std::endl;
}

void RunDijkstra(
    const Graph2D& graph,
    const Node2DPtr& start_node,
    const Node2DPtr& end_node) {
  
  std::cout << "==================================" << std::endl;
  std::cout << "======== RUNNING DIJKSTRA ========" << std::endl;
  std::cout << "==================================" << std::endl;

  // Run Dijkstra
  Dijkstra2D dijkstra;
  Dijkstra2D::Work work = dijkstra.Run(graph, start_node, end_node);

  // Print the solution
  work.statistics.Print();

  std::cout << "===== PATH =====" << std::endl;
  for(const Node2DPtr& node: work.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;
}

void RunAStar(
    const Graph2D& graph,
    const Node2DPtr& start_node,
    const Node2DPtr& end_node) {
  
  std::cout << "==================================" << std::endl;
  std::cout << "======== RUNNING A* ==============" << std::endl;
  std::cout << "==================================" << std::endl;

  // Run Dijkstra
  // Dijkstra2D dijkstra;
  // Dijkstra2D::Work work = dijkstra.Run(graph, start_node, end_node);

  // // Print the solution
  // work.statistics.Print();

  // std::cout << "===== PATH =====" << std::endl;
  // for(const Node2DPtr& node: work.path) {
  //   std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  // }

  // std::cout << std::endl;
}
