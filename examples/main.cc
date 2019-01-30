// Author: Tucker Haydon

#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

#include "node.h"
#include "directed_edge.h"
#include "graph.h"
#include "dijkstra.h"
#include "occupancy_grid2d.h"
#include "a_star.h"
#include "gnu_visualizer.h"

using namespace path_planning;


std::ostream & operator<<(std::ostream& out, const Node& node) {
  const int* data = reinterpret_cast<const int*>(node.Data());
  out << data[0] << ", " << data[1];
}

void RunAStar(const Graph& graph, 
              const OccupancyGrid2D& occupancy_grid,
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
                 const OccupancyGrid2D& occupancy_grid,
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
    const int start_coordinate[2] = {std::stoi(argv[1]), std::stoi(argv[2])};
    const int end_coordinate[2] = {std::stoi(argv[3]), std::stoi(argv[4])};
    const std::string file_path = argv[5];

    const size_t data_size = 2*sizeof(int);
    Node start, end;
    start.SetData(reinterpret_cast<const uint8_t*>(start_coordinate), data_size);
    end.SetData(reinterpret_cast<const uint8_t*>(end_coordinate), data_size);

    OccupancyGrid2D occupancy_grid;
    occupancy_grid.LoadFromFile(file_path);
    Graph graph(occupancy_grid);

    RunDijkstra(graph, occupancy_grid, start, end);
    RunAStar(graph, occupancy_grid, start, end);
  } else {
    std::cout << "Did you forget command line args?" << std::endl;
  }

  return 0;
}

