// Author: Tucker Haydon


#include <cstdlib>
#include <iostream>
#include <Eigen/StdVector>

#undef NDEBUG
#include <cassert>

#include "trajectory.h"
#include "trajectory_warden.h"
#include "quad_state.h"
#include "state_warden.h"

#include "point3d_potential.h"
#include "line2d_potential.h"
#include "point2d_potential.h"
#include "plane3d_potential.h"

using namespace mediation_layer;

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

void test_Point3DPotential() {
  {
    Point3D point(0,0,0);
    Point3DPotential::Options options;
    Point3DPotential potential(point, options);
  }
}

void test_TrajectoryWarden2D() {
  { // Trivial
    TrajectoryWarden2D warden;

    Trajectory2D dummy_trajectory;
    assert(0 == warden.Keys().size());
    assert(false == warden.Read("", dummy_trajectory));
    assert(false == warden.Write("", dummy_trajectory));
    assert(false == warden.Await("", dummy_trajectory));
  }

  { // Test read/write
    TrajectoryWarden2D warden;

    Trajectory2D trajectory_write({Eigen::Vector<double, 8>(1,1,1,1,1,1,1,1)});
    assert(true == warden.Register("test"));
    assert(true == warden.Write("test", trajectory_write));

    Trajectory2D trajectory_read;
    assert(true == warden.Read("test", trajectory_read));
    assert(trajectory_read.Size() == trajectory_write.Size());
    assert(trajectory_read.PVAYT(0).isApprox(trajectory_write.PVAYT(0)));
  }

}

void test_Trajectory2D() {
  { // Trivial
    Trajectory2D trajectory;
    assert(0 == trajectory.Size());
  }

  { // Test access
    TrajectoryVector2D hist = {};
    hist.push_back(Eigen::Vector<double, 8>(1,1,2,2,3,3,0.1,0.2));
    Trajectory2D trajectory(hist);

    assert(Eigen::Vector2d(1,1).isApprox(trajectory.Position(0)));
    assert(Eigen::Vector2d(2,2).isApprox(trajectory.Velocity(0)));
    assert(Eigen::Vector2d(3,3).isApprox(trajectory.Acceleration(0)));
    assert(0.1 == trajectory.Yaw(0));
    assert(0.2 == trajectory.Time(0));
    assert((Eigen::Vector<double, 6>(1,1,2,2,3,3)).isApprox(trajectory.PVA(0)));
    assert((Eigen::Vector<double, 8>(1,1,2,2,3,3,0.1,0.2)).isApprox(trajectory.PVAYT(0)));
  }
}

void test_QuadState2D() {
  { // Trivial
    QuadState2D state(Eigen::Vector<double,11>(1,1,2,2,1,0,0,0,1,2,3));
    assert((Eigen::Vector<double, 2>(1,1)).isApprox(state.Position()));
    assert((Eigen::Vector<double, 2>(2,2)).isApprox(state.Velocity()));
    assert((Eigen::Vector<double, 4>(1,0,0,0)).isApprox(state.Orientation()));
    assert((Eigen::Vector<double, 3>(1,2,3)).isApprox(state.Twist()));
  }
}

void test_StateWarden2D() {
  { // Trivial
    StateWarden2D warden;

    QuadState2D dummy_state;
    assert(0 == warden.Keys().size());
    assert(false == warden.Read("", dummy_state));
    assert(false == warden.Write("", dummy_state));
    assert(false == warden.Await("", dummy_state));
  }

  { // Test read/write
    StateWarden2D warden;

    QuadState2D state_write({Eigen::Vector<double, 11>(0,0,0,0,1,0,0,0,0,0,0)});
    assert(true == warden.Register("test"));
    assert(true == warden.Write("test", state_write));

    QuadState2D state_read;
    assert(true == warden.Read("test", state_read));
    assert(state_read.Position().isApprox(state_write.Position()));
    assert(state_read.Velocity().isApprox(state_write.Velocity()));
    assert(state_read.Orientation().isApprox(state_write.Orientation()));
    assert(state_read.Twist().isApprox(state_write.Twist()));
  }
}

void test_Plane3DPotential() {
  { // No force outside of range
    Plane3D plane({
        Line3D(Point3D(0,0,0), Point3D(1,0,0)),
        Line3D(Point3D(1,0,0), Point3D(1,1,0)),
        Line3D(Point3D(1,1,0), Point3D(0,1,0)),
        Line3D(Point3D(0,0,0), Point3D(0,0,0)),
        });
    Plane3DPotential::Options options;
    options.activation_dist = 1;
    options.min_dist = 0.2;
    const Plane3DPotential potential(plane, options);
    const Point3D p(0.5, 0.5, 100);
    const Vec3D force = potential.Resolve(p);
    assert(true == force.isApprox(Vec3D(0,0,0)));
  }

  { // Testing for inside and outside of convex region
    Plane3D plane({
        Line3D(Point3D(0,0,0), Point3D(1,0,0)),
        Line3D(Point3D(1,0,0), Point3D(1,1,0)),
        Line3D(Point3D(1,1,0), Point3D(0,1,0)),
        Line3D(Point3D(0,0,0), Point3D(0,0,0)),
        });
    Plane3DPotential::Options options;
    options.activation_dist = 1;
    options.min_dist = 0.2;
    const Plane3DPotential potential(plane, options);
    { // Outside
      const Point3D p(0.5, -0.5, 0.5);
      const Vec3D force = potential.Resolve(p);
      assert(true == force.isApprox(Vec3D(0,0,0)));
    }

    { // Inside
      const Point3D p(0.5, 0.5, 0.5);
      const Vec3D force = potential.Resolve(p);
      assert(false == force.isApprox(Vec3D(0,0,0)));
    }
  }

  { // Deterministic force
    Plane3D plane({
        Line3D(Point3D(0,0,0), Point3D(1,0,0)),
        Line3D(Point3D(1,0,0), Point3D(1,1,0)),
        Line3D(Point3D(1,1,0), Point3D(0,1,0)),
        Line3D(Point3D(0,0,0), Point3D(0,0,0)),
        });
    Plane3DPotential::Options options;
    options.activation_dist = 1;
    options.min_dist = 0.0;
    options.scale = 1.0;
    const Plane3DPotential potential(plane, options);
    { // Outside
      const Point3D p(0.5, 0.5, 0.5);
      const Vec3D force = potential.Resolve(p);
      assert(true == force.isApprox(Vec3D(0,0,3)));
    }
  }

}

int main(int argc, char** argv) {
  test_Trajectory2D();
  test_TrajectoryWarden2D();
  test_QuadState2D();
  test_StateWarden2D();
  test_Point3DPotential();
  test_Line2DPotential();
  test_Point2DPotential();
  test_Plane3DPotential();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
