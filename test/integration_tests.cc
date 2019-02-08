// Author: Tucker Haydon

#include <cstdlib>
#include <functional>
#include <cmath>
#include <iostream>

#include <Eigen/Core>

#include "runge_kutta_4.h"
#include "time_span.h"

using namespace path_planning;

void TestRK4Eigen(double epsilon) {
  
  const std::function<Eigen::Vector3d (double, const Eigen::Vector3d&)> f 
    = [](double time, const Eigen::Vector3d& val){ 
      return Eigen::Vector3d(time, std::pow(time, 2), std::pow(time, 3)); };

  const TimeSpan ts(0, 1, 1e-3);
  const Eigen::Vector3d x0(0, 0, 0);
  const RungeKutta4<Eigen::Vector3d> rk4;
  const Eigen::Vector3d x = 
    rk4.ForwardIntegrate(f, x0, ts);

  const Eigen::Vector3d expected_x(0.5, 1.0/3.0, 0.25);
  const Eigen::Vector3d diff = (x - expected_x);
  assert(std::abs(diff(0)) < epsilon);
  assert(std::abs(diff(1)) < epsilon);
  assert(std::abs(diff(2)) < epsilon);
}

void TestRK4Double(double epsilon) {

  // Function for y=x
  const std::function<double (double, double)> f 
    = [](double time, double val){ return time; };

  const TimeSpan ts(0, 1, 1e-3);
  const double x0(0);
  const RungeKutta4<double> rk4;
  const double x = 
    rk4.ForwardIntegrate(f, x0, ts);

  const double expected_x = 0.5;
  assert(std::abs(x - expected_x) < epsilon);
}

int main(int argc, char** argv) {

  const double EPSILON = 1e-3;
  TestRK4Double(EPSILON);
  TestRK4Eigen(EPSILON);
  std::cout << "All tests passed!" << std::endl;

  return EXIT_SUCCESS;
}
