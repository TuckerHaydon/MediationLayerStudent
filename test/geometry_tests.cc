// Author: Tucker Haydon

// Prevent assert from being optimized out
#undef NDEBUG

#include <cassert>
#include <iostream>
#include <Eigen/Core>

#include "line2d.h"
#include "line3d.h"
#include "polygon.h"
#include "line3d.h"
#include "plane3d.h"
#include "polyhedron.h"
#include "yaml-cpp/yaml.h"

using namespace mediation_layer;

void test_Polyhedron() {
  std::vector<Plane3D> faces(6);

  {
    const Point3D a(0,0,0), b(1,0,0), c(1,1,0), d(0,1,0);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    faces[0] = Plane3D({l1,l2,l3,l4});
  }

  {
    const Point3D a(0,0,1), b(0,1,1), c(1,1,1), d(1,0,1);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    faces[1] = Plane3D({l1,l2,l3,l4});
  }

  {
    const Point3D a(0,0,0), b(0,1,0), c(0,1,1), d(0,0,1);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    faces[2] = Plane3D({l1,l2,l3,l4});
  }

  {
    const Point3D a(0,0,0), b(0,0,1), c(1,0,1), d(1,0,0);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    faces[3] = Plane3D({l1,l2,l3,l4});
  }

  {
    const Point3D a(1,0,0), b(1,0,1), c(1,1,1), d(1,1,0);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    faces[4] = Plane3D({l1,l2,l3,l4});
  }

  {
    const Point3D a(0,1,0), b(1,1,0), c(1,1,1), d(0,1,1);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    faces[5] = Plane3D({l1,l2,l3,l4});
  }
 
  const Polyhedron poly(faces);

  const Point3D interior_point(0.5,0.5,0.5);
  const Point3D exterior_point(-0.5,-0.5,-0.5);

  assert(true  == poly.Contains(interior_point));
  assert(false == poly.Contains(exterior_point));

}

void test_Plane3D() { 
  { // Left side
    const Point3D a(0,0,0), b(1,0,0), c(1,1,0), d(0,1,0);
    const Line3D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    const Plane3D plane({l1,l2,l3,l4});

    const Point3D left_side_point(0.5,0.5,1);
    const Point3D right_side_point(0.5,0.5,-1);

    assert(true  == plane.OnLeftSide(left_side_point));
    assert(false == plane.OnLeftSide(right_side_point));
  }

  { // Read from file
    const std::string plane3d_yaml = R"V0G0N(
      [
        [0,0,0,1,0,0],
        [1,0,0,1,1,0],
        [1,1,0,0,1,0],
        [0,1,0,0,0,0]
      ])V0G0N";

    YAML::Node node = YAML::Load(plane3d_yaml);
    const Plane3D plane3d = node.as<Plane3D>();

    const Point3D left_side_point(0.5,0.5,1);
    const Point3D right_side_point(0.5,0.5,-1);

    assert(true  == plane3d.OnLeftSide(left_side_point));
    assert(false == plane3d.OnLeftSide(right_side_point));
  }
}

void test_Polygon() {
  { // Contains
    const Point2D a(0, 0), b(1,0), c(1,1), d(0,1);
    const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    const Polygon poly({l1,l2,l3,l4});

    const Point2D interior_point(0.5,0.5);
    const Point2D exterior_point(-1,1);

    assert(true  == poly.Contains(interior_point));
    assert(false == poly.Contains(exterior_point));
  }

  { // Expand and shrink
    const Point2D a(0, 0), b(1,0), c(1,1), d(0,1);
    const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a);
    const Polygon poly({l1,l2,l3,l4});

    const Point2D shrink_to_exclude_point(0.9,0.9);
    const Point2D expand_to_include_point(1.1,1.1);

    assert(true == poly.Contains(shrink_to_exclude_point));
    assert(false == poly.Shrink(0.2).Contains(shrink_to_exclude_point));
    assert(false == poly.Contains(expand_to_include_point));
    assert(true == poly.Expand(0.2).Contains(expand_to_include_point));
  }

  { // IsConvex
    const Point2D a(0, 0), b(1,0), c(1,1), d(0,1), e(0.5,0.5);
    const Line2D l1(a,b), l2(b,c), l3(c,d), l4(d,a), l5(c,e), l6(e,d);
    const Polygon convex_poly({l1,l2,l3,l4});
    const Polygon non_convex_poly({l1,l2,l5,l6,l4});

    assert(true == convex_poly.IsConvex());
    assert(false == non_convex_poly.IsConvex());
  }

  { // Construct from points
    const Point2D a(0, 0), b(1,0), c(1,1), d(0,1);
    Polygon poly;
    poly.ConvexHullFromPoints({a,c,d,b});

    const Point2D interior_point(0.5,0.5);
    const Point2D exterior_point(-1,1);

    assert(true  == poly.Contains(interior_point));
    assert(false == poly.Contains(exterior_point));
  }

  { // Read from file
    const std::string polygon_yaml = R"V0G0N(
      polygon: [
        [0, 0, 1, 0],
        [1, 0, 1, 1],
        [1, 1, 0, 1],
        [0, 1, 0, 0]
      ])V0G0N";

    YAML::Node node = YAML::Load(polygon_yaml);
    const Polygon polygon = node["polygon"].as<Polygon>();
    assert(4 == polygon.Vertices().size());
    assert(1 == polygon.Vertices()[2].x());
    assert(1 == polygon.Vertices()[2].y());
  }
}

void test_Line2D() {
  { // OnLeftSide
    const Point2D a(0, 0), b(1,1), c(0,1);
    const Line2D l(a,b);

    assert(true == l.OnLeftSide(c));
  }

  { // Standard Equation
    const Point2D a(0, 0), b(1,1), c(1,2), d(2,3), e(2,0);
    const Line2D l1(a,b), l2(c,d), l3(a,e);

    std::pair<Point2D, double> se1 = l1.StandardForm();
    assert(0 == se1.second);
  }

  { // Intersection point
    const Point2D a(0, 0), b(1,1), c(2,0);
    const Line2D l1(a,b), l2(a,c), l3(b,c);

    const Point2D i1 = l1.IntersectionPoint(l2);
    assert(0 == i1.x());
    assert(0 == i1.y());

    const Point2D i2 = l2.IntersectionPoint(l3);
    assert(2 == i2.x());
    assert(0 == i2.y());
  }

  { // Normal intersection point
    const Point2D a(0, 0), b(1,1), c(1,0);
    const Line2D l(a,b);

    const Point2D i = l.NormalIntersectionPoint(c);
    assert(0.5 == i.x());
    assert(0.5 == i.y());
  }

  { // Contains
    const Point2D a(1,1), b(3,5), c(2,3), d(0,0), e(4,7);
    const Line2D l(a,b);

    assert(true == l.Contains(c));
    assert(false == l.Contains(d));
    assert(false == l.Contains(e));
  }

  { // ProjectedContains
    const Point2D a(1,1), b(3,5), c(2,2), d(0,0), e(4,7), f(0,-1), g(2,3), h(2,1);
    const Line2D l(a,b);

    assert(true  == l.ProjectedContains(c));
    assert(false == l.ProjectedContains(d));
    assert(false == l.ProjectedContains(e));
    assert(false == l.ProjectedContains(f));
    assert(true  == l.ProjectedContains(g));
    assert(true  == l.ProjectedContains(h));
  }

  { // Orthogonal Unit Vector
    const Point2D a(0,0), b(1,1), c(-1, 1), d(-1,-1), e(1,-1);

    {
      const Line2D l(a,b);
      assert(l.OrthogonalUnitVector().isApprox(Point2D(-std::sqrt(2)/2,std::sqrt(2)/2)));  
    }
    {
      const Line2D l(a,c);
      assert(l.OrthogonalUnitVector().isApprox(Point2D(-std::sqrt(2)/2,-std::sqrt(2)/2)));  
    }
    {
      const Line2D l(a,d);
      assert(l.OrthogonalUnitVector().isApprox(Point2D(std::sqrt(2)/2,-std::sqrt(2)/2)));  
    }
    {
      const Line2D l(a,e);
      assert(l.OrthogonalUnitVector().isApprox(Point2D(std::sqrt(2)/2,std::sqrt(2)/2)));  
    }
  }

  { // Read from file
    YAML::Node node = YAML::Load("test_line: [0, 0, 1, 2]");
    const Line2D line = node["test_line"].as<Line2D>();
    assert(0 == line.Start().x());
    assert(0 == line.Start().y());
    assert(1 == line.End().x());
    assert(2 == line.End().y());
  }
}

void test_Line3D() {
  { // Test AsVector
    Line3D l(Point3D(1,1,1), Point3D(2,2,2));
    Point3D v = l.AsVector();
    assert(Point3D(1,1,1).isApprox(v)); 
  }

  { // Read from file
    const std::string line3d_yaml = R"V0G0N(
      [
      1, 2, 3, 4, 5, 6
      ])V0G0N";

    YAML::Node node = YAML::Load(line3d_yaml);
    const Line3D line3d = node.as<Line3D>();
    assert(1 == line3d.Start().x());
    assert(2 == line3d.Start().y());
    assert(3 == line3d.Start().z());
    assert(4 == line3d.End().x());
    assert(5 == line3d.End().y());
    assert(6 == line3d.End().z());
  }
}

int main(int argc, char** argv) {
  test_Line2D();
  test_Line3D();
  test_Polygon();
  test_Plane3D();
  test_Polyhedron();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
