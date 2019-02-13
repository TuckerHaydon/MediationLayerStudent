// Author: Tucker Haydon

#include <cstdlib>
#include <vector>

#include "safe_flight_corridor2d.h"
#include "map2d.h"
#include "gui2d.h"
#include "graph.h"
#include "a_star.h"
#include "dijkstra.h"
#include "timer.h"

using namespace path_planning;

int main(int argc, char** argv) {
  Map2D map;
  {
    Polygon boundary;
    {
      const Point2D a(0,0), b(10,0), c(10,10), d(0,10);
      boundary.ConstructFromPoints({a,b,c,d});
    }

    std::vector<Polygon> obstacles;

    {
      const Point2D a(5,2), b(6,2), c(6,3), d(5,3);
      Polygon obstacle;
      obstacle.ConstructFromPoints({a,b,c,d});
      obstacles.push_back(obstacle);
    }

    {
      const Point2D a(2,4), b(3,4), c(3,10), d(2,10);
      Polygon obstacle;
      obstacle.ConstructFromPoints({a,b,c,d});
      obstacles.push_back(obstacle);
    }

    {
      const Point2D a(7,4), b(8,4), c(8,5), d(7,5);
      Polygon obstacle;
      obstacle.ConstructFromPoints({a,b,c,d});
      obstacles.push_back(obstacle);
    }

    map.SetObstacles(obstacles);
    map.SetBoundary(boundary);
  }

  const double SAFETY_BOUND = 0.3;
  const double SAMPLE_DELTA = 0.5;

  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromMap(map, SAMPLE_DELTA, SAFETY_BOUND);
  // Graph graph(occupancy_grid);

  // const size_t data_size = 2*sizeof(int);
  // const double start[2] = {9,1}, end[2] = {9,9};
  // const int start_int[2] = {(int)(start[0]/SAMPLE_DELTA),(int)(start[1]/SAMPLE_DELTA)};
  // const int end_int[2] = {(int)(end[0]/SAMPLE_DELTA),(int)(end[1]/SAMPLE_DELTA)};

  // Node start_node, end_node;
  // start_node.SetData(reinterpret_cast<const uint8_t*>(start_int), data_size);
  // end_node.SetData(reinterpret_cast<const uint8_t*>(end_int), data_size);

  // const std::vector<Node> node_path = AStar().Run(graph, start_node, end_node);

  // {
  //   std::vector<Point2D> path;
  //   path.reserve(node_path.size());
  //   // Rows, cols to x, y
  //   std::for_each(
  //     node_path.begin(),
  //     node_path.end(),
  //     [&](const Node& node){
  //       path.emplace_back(
  //           ((int*)node.Data())[1],
  //           ((int*)node.Data())[0]);
  //     });

  //   Gui2D gui;
  //   gui.LoadOccupancyGrid(&occupancy_grid);
  //   gui.LoadPath(path);
  //   gui.Display();
  // }


  // {
  //   std::vector<Point2D> path;
  //   path.reserve(node_path.size());
  //   // Rows, cols to x, y
  //   std::for_each(
  //       node_path.begin(),
  //       node_path.end(),
  //       [&](const Node& node){
  //         path.emplace_back(
  //             ((int*)node.Data())[1] * SAMPLE_DELTA,
  //             ((int*)node.Data())[0] * SAMPLE_DELTA);
  //       });

  //   SafeFlightCorridor2D sfc2d;
  //   sfc2d.SetMap(map);
  //   auto timer = Timer("SFC timer finished");
  //   timer.Start();
  //   std::vector<Polygon> sfc = sfc2d.Run(path);
  //   timer.Stop();

  //   Gui2D gui;
  //   gui.LoadMap(map);
  //   gui.LoadSafeFlightCorridor(sfc);
  //   gui.LoadPath(path);
  //   gui.Display();
  // }

  return EXIT_SUCCESS;
}
