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

    Polygon obstacle1;
    {
      const Point2D a(3,-4), b(4,-4), c(4,-3), d(3,-3);
      obstacle1.ConstructFromPoints({a,b,c,d});
    }

    Polygon obstacle2;
    {
      const Point2D a(-3,-1), b(-2,-1), c(-2,5), d(-3,5);
      obstacle2.ConstructFromPoints({a,b,c,d});
    }

    map.SetObstacles({obstacle1, obstacle2});
    map.SetBoundary(boundary);
  }

  std::vector<Point2D> path = {Point2D(-1, -1), Point2D(1,1)};

  SafeFlightCorridor2D sfc2d;
  sfc2d.SetMap(map);
  std::vector<Polygon> sfc = sfc2d.Run(path);

  return EXIT_SUCCESS;
}
