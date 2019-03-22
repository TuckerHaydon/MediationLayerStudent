// Author: Tucker Haydon

#include <cstdlib>

#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"

using namespace mediation_layer;
using Node2DPtr = std::shared_ptr<Node2D>;

int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file x1 y1 x2 y2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const Node2DPtr start_node = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const Node2DPtr end_node = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  return EXIT_SUCCESS;
}
