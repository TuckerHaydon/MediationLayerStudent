// Author: Tucker Haydon

#include <cstdlib>
#include <vector>

#include "polynomial_solver.h"

using namespace mediation_layer;

///////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void Example();
void DerivativeExperiments();
void ArrivalTimeExperiments();
void NumWaypointExperiments();

///////////////////////////////////////////////////////////////////
// MAIN FUNCTION
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  Example();
  return EXIT_SUCCESS;
}

void Example() {
  const std::vector<double> times = {0, 0.5, 1, 1.5};

  // Equality bounds paramater order is:
  // 1) Dimension index
  // 2) Node index
  // 3) Derivative index
  // 4) Value
  const std::vector<NodeEqualityBound> node_equality_bounds = {
    // The first node must constrain position, velocity, and acceleration
    NodeEqualityBound(0,0,0,0),
    NodeEqualityBound(1,0,0,0),
    NodeEqualityBound(0,0,1,0),
    NodeEqualityBound(1,0,1,0),
    NodeEqualityBound(0,0,2,0),
    NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    NodeEqualityBound(0,1,0,1),
    NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    NodeEqualityBound(0,2,0,1),
    NodeEqualityBound(1,2,0,1),

    // The fourth node constrains position
    NodeEqualityBound(0,3,0,0),
    NodeEqualityBound(1,3,0,0),
  };

  PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 3D
  solver_options.polynomial_order = 7;   // Fit an 7th-order polynomial
  solver_options.continuity_order = 3;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize the 4th order (snap)
  solver_options.polish = true;          // Polish the solution

  PolynomialSolver solver(solver_options);
  const PolynomialPath path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});
}
