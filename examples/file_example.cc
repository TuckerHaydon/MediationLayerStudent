// Author: Tucker Haydon

#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "node2d.h"
#include "graph.h"
#include "occupancy_grid2d.h"
#include "dijkstra.h"

using namespace mediation_layer;

int main(int argc, char** argv) {
  if(argc == 6) {
    const double start_coordinate[2] = {std::stod(argv[1]), std::stod(argv[2])};
    const double end_coordinate[2] = {std::stod(argv[3]), std::stod(argv[4])};
    const std::string file_path = argv[5];

    const auto start_node = std::make_shared<Node2D>(Eigen::Vector2d(
        start_coordinate[0],
        start_coordinate[1]));
    const auto end_node = std::make_shared<Node2D>(Eigen::Vector2d(
        end_coordinate[0],
        end_coordinate[1]));

    OccupancyGrid2D occupancy_grid;
    occupancy_grid.LoadFromFile(file_path);
    const Graph2D graph = occupancy_grid.AsGraph();
    const Dijkstra2D::Path path = Dijkstra2D().Run(graph, start_node, end_node);
    path.statistics.Print();

  } else {
    std::cout << "Did you forget command line args?" << std::endl;
  }

  return 0;
}

