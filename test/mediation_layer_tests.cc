// Author: Tucker Haydon


#include <cstdlib>
#include <iostream>

#undef NDEBUG
#include <cassert>

#include "state2d.h"
#include "trajectory2d.h"
#include "line2d_potential.h"
#include "point2d_potential.h"

using namespace path_planning;

void test_Point2DPotential() {
  { // No force outside of range
    const Point2D a(0,0), b(1,1);
    Point2DPotential::Options options;
    options.activation_dist = 1;
    options.min_dist = 0.2;
    const Point2DPotential potential(a, options);
    const Vec2D force = potential.Resolve(b);
    assert(force.isApprox(Vec2D(0,0)));
  }

  { // Deterministic force
    const Point2D a(0,0), b(0.5,0);
    Point2DPotential::Options options;
    options.activation_dist = 1;
    options.min_dist = 0.0;
    options.scale = 1.0;
    const Point2DPotential potential(a, options);
    const Vec2D force = potential.Resolve(b);
    assert(force.isApprox(Vec2D(3,0)));
  }
}

void test_Line2DPotential() {
  { // No force behind
    const Point2D a(0,0), b(1,0), c(1,-1);
    const Line2D line(a,b);
    Line2DPotential::Options options; 
    options.activate_behind = false;
    const Line2DPotential potential(line, options);
    const Vec2D force = potential.Resolve(c);
    assert(force.isApprox(Vec2D(0,0)));
  }
  { // No force on side
    const Point2D a(0,0), b(1,0), c(-1,1);
    const Line2D line(a,b);
    Line2DPotential::Options options; 
    const Line2DPotential potential(line, options);
    const Vec2D force = potential.Resolve(c);
    assert(force.isApprox(Vec2D(0,0)));
  }
  { // Force in front
    const Point2D a(0,0), b(2,0), c(1,1), d(1,2), e(1,0.75);
    const Line2D line(a,b);
    Line2DPotential::Options options; 
    options.scale = 1;
    options.activation_dist = 1;
    options.min_dist = 0.5;
    const Line2DPotential potential(line, options);
    assert(potential.Resolve(c).isApprox(Vec2D(0,0)));
    assert(potential.Resolve(d).isApprox(Vec2D(0,0)));
    assert(potential.Resolve(e).isApprox(Vec2D(0,12)));
  }
}

void test_State2D() {
  { // Read non-exist returns false
    State2D state;
    Trajectory2D trajectory;
    assert(false == state.Read("", trajectory));
  }

  { // Write non-exist returns false  
    State2D state;
    Trajectory2D trajectory;
    assert(false == state.Write("", trajectory));
  }
}

int main(int argc, char** argv) {
  test_State2D();
  test_Line2DPotential();
  test_Point2DPotential();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
