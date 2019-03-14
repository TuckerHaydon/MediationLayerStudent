// Author: PUT YOUR NAME HERE

#include <cstdlib>

#include "dijkstra_student.h"

using namespace mediation_layer;

int main(int argc, char** argv) {
  Graph2D graph;
  std::shared_ptr<Node2D> start_node = std::make_shared<Node2D>(Eigen::Vector2d(0,0));
  std::shared_ptr<Node2D> end_node = std::make_shared<Node2D>(Eigen::Vector2d(1,1));

  Dijkstra2D dijkstra;
  dijkstra.Run(graph, start_node, end_node);

  return EXIT_SUCCESS;
}
