// Author: Tucker Haydon

#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

#include "node.h"
#include "directed_edge.h"
#include "undirected_edge.h"
#include "graph.h"
#include "dijkstra.h"
#include "occupancy_grid.h"
#include "a_star.h"

using namespace path_planning;

void RunAStar(const Graph* graph, const Node& start, const Node& end) {

    const AStar a_star(graph);
    std::vector<Node> path = a_star.Run(start, end);

    std::cout << "Path:" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node.id_ << std::endl;
        });
}

void RunDijkstra(const Graph* graph, const Node& start, const Node& end) {

    const Dijkstra dijkstra(graph);
    std::vector<Node> path = dijkstra.Run(start, end);

    std::cout << "Path:" << std::endl;
    std::for_each(
        path.begin(),
        path.end(),
        [](const Node& node){
          std::cout << node.id_ << std::endl;
        });
}

int main(int argc, char** argv) {
  // const Node a("a"), b("b"), c("c"), d("d"), e("e"), f("f"), g("g");

  // const DirectedEdge e1(a, b, 1), 
  //                    e2(a, c, 2), 
  //                    e3(b, d, 3), 
  //                    e4(c, e, 2), 
  //                    e5(d, f, 1), 
  //                    e6(e, f, 1);

  // const UndirectedEdge u1(f, g, 0);

  // Graph graph;
  // graph.AddEdges({e1, e2, e3, e4, e5, e6});
  // graph.AddEdges({u1});
  
  if(argc != 1) {    
    std::string file_path = argv[1];

    Node start("(0,0)"), end("(4,4)");

    OccupancyGrid occupancy_grid(file_path);
    Graph graph = occupancy_grid.ToGraph();

    RunDijkstra(&graph, start, end);
    RunAStar(&graph, start, end);
  }

  return 0;
}
