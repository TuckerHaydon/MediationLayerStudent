// Author: Tucker Haydon


#include <cstdlib>
#include <iostream>

#undef NDEBUG
#include <cassert>

#include "state2d.h"
#include "trajectory2d.h"
#include "line2d_force.h"

using namespace path_planning;

void test_Line2DForce() {
  { // No force behind
    const Point2D a(0,0), b(1,0), c(1,-1);
    const Line2D line(a,b);
    Line2DForce::Options options; 
    options.activate_behind = false;
    const Line2DForce force_field(line, options);
    const Vec2D force = force_field.Resolve(c);
    assert(force.isApprox(Vec2D(0,0)));
  }
  { // No force on side
    const Point2D a(0,0), b(1,0), c(-1,1);
    const Line2D line(a,b);
    Line2DForce::Options options; 
    const Line2DForce force_field(line, options);
    const Vec2D force = force_field.Resolve(c);
    assert(force.isApprox(Vec2D(0,0)));
  }
  { // Force in front
    const Point2D a(0,0), b(2,0), c(1,1), d(1,2), e(1,0.75);
    const Line2D line(a,b);
    Line2DForce::Options options; 
    options.scale = 1;
    options.activation_dist = 1;
    options.min_dist = 0.5;
    const Line2DForce force_field(line, options);
    assert(force_field.Resolve(c).isApprox(Vec2D(0,0)));
    assert(force_field.Resolve(d).isApprox(Vec2D(0,0)));
    assert(force_field.Resolve(e).isApprox(Vec2D(0,12)));
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
  test_Line2DForce();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
