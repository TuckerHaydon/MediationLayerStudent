// Author: Tucker Haydon

#undef NDEBUG
#include <cassert>

#include <iostream>

#include "dijkstra.h"
#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "node2d.h"

using namespace mediation_layer;

void test_Dijkstra2D() {
  // Array
  const bool array[4][4] = {
    {0,0,0,1},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,0,0}
  };

  // Allocate buffer
  bool** buffer = (bool**) std::malloc(4*sizeof(bool*));
  for(size_t idx = 0; idx < 4; ++idx) {
    buffer[idx] = (bool*) std::malloc(4*sizeof(bool));
  }

  // Copy array into buffer
  for(size_t row = 0; row < 4; ++row) {
    for (size_t col = 0; col < 4; ++col) {
      buffer[row][col] = array[row][col];
    }
  }

  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromBuffer(const_cast<const bool**>(buffer), 4, 4);
  const Graph2D graph = occupancy_grid.AsGraph();

  auto start = std::make_shared<Node2D>(Eigen::Vector2d(0,0));
  auto end   = std::make_shared<Node2D>(Eigen::Vector2d(3,3));
  Dijkstra2D dijkstra;
  dijkstra.Run(graph, start, end);
  Dijkstra2D::Path path = Dijkstra2D().Run(graph, start, end);
  path.statistics.Print();
  assert(5 == path.statistics.path_length);
}

void test_AStar2D() {
  // Array
  const bool array[4][4] = {
    {0,0,0,1},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,0,0}
  };

  // Allocate buffer
  bool** buffer = (bool**) std::malloc(4*sizeof(bool*));
  for(size_t idx = 0; idx < 4; ++idx) {
    buffer[idx] = (bool*) std::malloc(4*sizeof(bool));
  }

  // Copy array into buffer
  for(size_t row = 0; row < 4; ++row) {
    for (size_t col = 0; col < 4; ++col) {
      buffer[row][col] = array[row][col];
    }
  }

  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromBuffer(const_cast<const bool**>(buffer), 4, 4);
  const Graph2D graph = occupancy_grid.AsGraph();

  auto start = std::make_shared<Node2D>(Eigen::Vector2d(0,0));
  auto end   = std::make_shared<Node2D>(Eigen::Vector2d(3,3));

  const std::function<double(const Node2D&, const Node2D&)> heuristic 
    = [](const Node2D& a, 
         const Node2D& b) { 
    return (a.Data() - b.Data()).norm(); };

  AStar2D::Path path = AStar2D().Run(graph, start, end);
  path.statistics.Print();
  assert(5 == path.statistics.path_length);
}


int main(int argc, char** argv) {
  test_Dijkstra2D();
  test_AStar2D();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
