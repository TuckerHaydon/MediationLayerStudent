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

using namespace path_planning;

void RunAStar(const Graph& graph, const Node& start, const Node& end) {

    const std::vector<Node> path = AStar().Run(graph, start, end);

    std::cout << "Path (" << path.size() << " nodes):" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node << std::endl;
        });
}

void RunDijkstra(const Graph& graph, const Node& start, const Node& end) {

    const std::vector<Node> path = Dijkstra().Run(graph, start, end);

    std::cout << "Path (" << path.size() << " nodes):" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node << std::endl;
        });
}

int main(int argc, char** argv) {
  if(argc == 6) {
    Node start({std::stoi(argv[1]), std::stoi(argv[2])});
    Node end({std::stoi(argv[3]), std::stoi(argv[4])});
    std::string file_path = argv[5];

    OccupancyGrid occupancy_grid(file_path);
    Graph graph(occupancy_grid);

    RunDijkstra(graph, start, end);
    RunAStar(graph, start, end);
  }

  return 0;
}

