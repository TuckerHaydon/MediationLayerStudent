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
          std::cout << node.id_ << std::endl;
        });
}

void RunDijkstra(const Graph& graph, const Node& start, const Node& end) {

    const std::vector<Node> path = Dijkstra().Run(graph, start, end);

    std::cout << "Path (" << path.size() << " nodes):" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node.id_ << std::endl;
        });
}

int main(int argc, char** argv) {
  if(argc == 4) {
    Node start{std::string(argv[1])};
    Node end{std::string(argv[2])};
    std::string file_path = argv[3];

    OccupancyGrid occupancy_grid(file_path);
    Graph graph(occupancy_grid);

    RunDijkstra(graph, start, end);
    RunAStar(graph, start, end);
  }

  return 0;
}
