// Author: Tucker Haydon

#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

#include "node.h"
#include "directed_edge.h"
#include "graph.h"
#include "dijkstra.h"
#include "occupancy_grid.h"
#include "a_star.h"
#include "gnu_visualizer.h"

using namespace path_planning;

void RunAStar(const Graph& graph, 
              const OccupancyGrid& occupancy_grid,
              const Node& start, 
              const Node& end) {

    const std::vector<Node> path = AStar().Run(graph, start, end);

    std::cout << "Path (" << path.size() << " nodes):" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node << std::endl;
        });

    GNUVisualizer().Run(occupancy_grid, path);
}

void RunDijkstra(const Graph& graph, 
                 const OccupancyGrid& occupancy_grid,
                 const Node& start, 
                 const Node& end) {

    const std::vector<Node> path = Dijkstra().Run(graph, start, end);

    std::cout << "Path (" << path.size() << " nodes):" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node << std::endl;
        });

    GNUVisualizer().Run(occupancy_grid, path);
}

int main(int argc, char** argv) {
  if(argc == 6) {
    Node start({std::stoi(argv[1]), std::stoi(argv[2])});
    Node end({std::stoi(argv[3]), std::stoi(argv[4])});
    std::string file_path = argv[5];

    OccupancyGrid occupancy_grid(file_path);
    Graph graph(occupancy_grid);

    RunDijkstra(graph, occupancy_grid, start, end);
    RunAStar(graph, occupancy_grid, start, end);
  } else {
    std::cout << "Did you forget command line args?" << std::endl;
  }

  return 0;
}

