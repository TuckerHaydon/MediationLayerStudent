// Author: Tucker Haydon

#include <vector>
#include <memory>

#include "graph.h"
#include "occupancy_grid2d.h"

using namespace mediation_layer;
using Node2DPtr = std::shared_ptr<Node2D>;

int main(int argc, char** argv) {
  if(argc != 2) {
    std::cerr << "Usage: ./examples ${occupancy_grid_file}" << std::endl;
    return EXIT_FAILURE;
  }

  // Load an occupancy grid from a file
  // There are two example grids at: data/labs/grid2d_*
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(argv[1]);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  // Define the start and end points
  // Create a node at (0,0) and a node at (4,4)
  const Node2DPtr start_node = std::make_shared<Node2D>(Eigen::Vector2d(0,0));
  const Node2DPtr end_node = std::make_shared<Node2D>(Eigen::Vector2d(4,4));

  // Access directed edges eminating from a node
  const std::vector<DirectedEdge2D> edges = graph.Edges(start_node);
  
  // Iterate through the list of edges
  for(const DirectedEdge2D& edge: edges) {
    const Node2DPtr& source_ptr = edge.Source();
    const Node2DPtr& sink_ptr = edge.Sink();
    const double cost = edge.Cost();

    // Print relevant data
    std::cout 
      << "DirectedEdge2D from " 
      << "[" << source_ptr->Data().transpose() << "]"
      << " to "
      << "[" << sink_ptr->Data().transpose() << "]"
      << " with cost " 
      << cost
      << std::endl;
  }

  // Check node equality
  std::cout << "Are the start and end nodes equal: " << (*start_node == *end_node) << std::endl;
  std::cout << "Is the start node equal to itself: " << (*start_node == *start_node) << std::endl;

  return EXIT_SUCCESS;
}
