// Author: Tucker Haydon

#undef NDEBUG
#include <cassert>

#include <iostream>

#include "occupancy_grid2d.h"
#include "node2d.h"

using namespace path_planning;

void test_Map2D() {
  { // Contains/Free Space
    Polygon boundary;
    {
      const Point2D a(0,0), b(10,0), c(10,10), d(0,10);
      const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
      boundary.SetEdges({l1,l2,l3,l4});
    }

    Polygon obstacle;
    {
      const Point2D a(4,4), b(6,4), c(6,6), d(4,6);
      const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
      obstacle.SetEdges({l1,l2,l3,l4});
    }

    const Map2D map(boundary, {obstacle});

    const Point2D point_inside_map(1,1);
    const Point2D point_outside_map(20,-1);
    const Point2D free_point(1,1);
    const Point2D obstacle_point(5,5);

    assert(true == map.Contains(point_inside_map));
    assert(false == map.Contains(point_outside_map));
    assert(true == map.IsFreeSpace(free_point));
    assert(false == map.IsFreeSpace(obstacle_point));
  }

  { // Inflate
    Polygon boundary;
    {
      const Point2D a(0,0), b(10,0), c(10,10), d(0,10);
      const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
      boundary.SetEdges({l1,l2,l3,l4});
    }

    Polygon obstacle;
    {
      const Point2D a(4,4), b(6,4), c(6,6), d(4,6);
      const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
      obstacle.SetEdges({l1,l2,l3,l4});
    }

    const Map2D map(boundary, {obstacle});
    const Map2D inflated_map = map.Inflate(0.5);

    const Point2D point_still_inside(1,1);
    const Point2D point_outside(0,0);
    const Point2D point_still_free(1,1);
    const Point2D point_not_free(3.9,3.9);

    assert(true == inflated_map.Contains(point_still_inside));
    assert(false == inflated_map.Contains(point_outside));
    assert(true == inflated_map.IsFreeSpace(point_still_free));
    assert(false == inflated_map.IsFreeSpace(point_not_free));
  }
}

void test_OccupancyGrid2D() {
  Polygon boundary;
  {
    const Point2D a(0,0), b(10,0), c(10,10), d(0,10);
    const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    boundary.SetEdges({l1,l2,l3,l4});
  }

  Polygon obstacle;
  {
    const Point2D a(4,4), b(6,4), c(6,6), d(4,6);
    const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    obstacle.SetEdges({l1,l2,l3,l4});
  }

  const Map2D map(boundary, {obstacle});

  const double sample_delta = 0.1;
  const double safety_bound = 0;
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromMap(map, sample_delta, safety_bound);
  const Graph2D graph = occupancy_grid.AsGraph();

  {
    const auto n = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(0,0));
    assert(3 == graph.Edges(n).size());
  }

  {
    const auto n = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(1,1));
    assert(8 == graph.Edges(n).size());
  }
}

int main(int argc, char** argv) {
  test_Map2D();
  test_OccupancyGrid2D();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
