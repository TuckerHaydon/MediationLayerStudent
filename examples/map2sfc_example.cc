// Author: Tucker Haydon

#include <cstdlib>
#include <vector>

#include "safe_flight_corridor2d.h"
#include "map2d.h"
#include "gui2d.h"
#include "graph.h"
#include "a_star2d.h"
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
  Graph2D graph = occupancy_grid.AsGraph();

  const size_t data_size = 2*sizeof(int);
  const double start[2] = {9,1}, end[2] = {9,9};
  const int start_int[2] = {(int)(start[0]/SAMPLE_DELTA),(int)(start[1]/SAMPLE_DELTA)};
  const int end_int[2] = {(int)(end[0]/SAMPLE_DELTA),(int)(end[1]/SAMPLE_DELTA)};

  const auto start_node = std::make_shared<Node2D>(Eigen::Vector2d(start_int[0], start_int[1]));
  const auto end_node = std::make_shared<Node2D>(Eigen::Vector2d(end_int[0], end_int[1]));
  const AStar2D::Path a_star_solution = AStar2D().Run(graph, start_node, end_node);

  {
    std::vector<Point2D> path;
    path.reserve(a_star_solution.nodes.size());
    std::for_each(
      a_star_solution.nodes.begin(),
      a_star_solution.nodes.end(),
      [&](const auto& node){
        path.emplace_back(
            // Rows, cols to x, y
            node->Data().y(),
            node->Data().x());
      });

    Gui2D gui;
    gui.LoadOccupancyGrid(&occupancy_grid);
    gui.LoadPath(path);
    gui.Display();
  }

  {
    std::vector<Point2D> path;
    path.reserve(a_star_solution.nodes.size());
    std::for_each(
      a_star_solution.nodes.begin(),
      a_star_solution.nodes.end(),
      [&](const auto& node){
        path.emplace_back(
            // Rows, cols to x, y
            node->Data().y() * SAMPLE_DELTA,
            node->Data().x() * SAMPLE_DELTA);
      });

    SafeFlightCorridor2D sfc2d;
    sfc2d.SetMap(map);
    auto timer = Timer("SFC timer finished");
    timer.Start();
    std::vector<Polygon> sfc = sfc2d.Run(path);
    timer.Stop();
    timer.Print();

    Gui2D gui;
    gui.LoadMap(map);
    gui.LoadSafeFlightCorridor(sfc);
    gui.LoadPath(path);
    gui.Display();
  }

  return EXIT_SUCCESS;
}
