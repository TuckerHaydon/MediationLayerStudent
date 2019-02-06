// Author: Tucker Haydon

#include <cstdlib>
#include <vector>

#include "safe_flight_corridor2d.h"
#include "map2d.h"

using namespace path_planning;
using namespace geometry;


int main(int argc, char** argv) {

  Map2D map;
  {
    Polygon boundary;
    {
      const Point2D a(-5,-5), b(5,-5), c(5,5), d(-5,5);
      boundary.ConstructFromPoints({a,b,c,d});
    }

    std::vector<Polygon> obstacles;

    {
      const Point2D a(0,-3), b(1,-3), c(1,-2), d(0,-2);
      Polygon obstacle;
      obstacle.ConstructFromPoints({a,b,c,d});
      obstacles.push_back(obstacle);
    }

    {
      const Point2D a(-3,-1), b(-2,-1), c(-2,5), d(-3,5);
      Polygon obstacle;
      obstacle.ConstructFromPoints({a,b,c,d});
      obstacles.push_back(obstacle);
    }

    {
      const Point2D a(2,-1), b(3,-1), c(3,0), d(2,0);
      Polygon obstacle;
      obstacle.ConstructFromPoints({a,b,c,d});
      obstacles.push_back(obstacle);
    }

    map.SetObstacles(obstacles);
    map.SetBoundary(boundary);
  }

  std::vector<Point2D> path = {Point2D(-1, -1), Point2D(1,1), Point2D(1,2)};

  SafeFlightCorridor2D sfc2d;
  sfc2d.SetMap(map);
  std::vector<Polygon> sfc = sfc2d.Run(path);

  return EXIT_SUCCESS;
}
