// Author: Tucker Haydon

#undef NDEBUG
#include <cassert>

#include <iostream>
#include <memory>

#include "node.h"
#include "node2d.h"
#include "node3d.h"
#include "directed_edge.h"
#include "graph.h"

using namespace mediation_layer;

void test_Node2D() {
  { // Check equality
    const Node2D n1(Eigen::Matrix<double, 2, 1>(1,2));
    const Node2D n2(Eigen::Matrix<double, 2, 1>(1,2));
    const Node2D n3(Eigen::Matrix<double, 2, 1>(1,3));
    assert(true == (n1 == n2));
    assert(false == (n1 == n3));
  }

  { // Check hash equality
    const Node2D n1(Eigen::Matrix<double, 2, 1>(1,2));
    const Node2D n2(Eigen::Matrix<double, 2, 1>(1,2));
    const Node2D n3(Eigen::Matrix<double, 2, 1>(1.0001,2.0001));
    assert(true == (n1.Hash() == n2.Hash()));
    assert(false == (n1.Hash() == n3.Hash()));
  }
}

void test_Node3D() {
  { // Check equality
    const Node3D n1(Eigen::Matrix<double, 3, 1>(1,2,1));
    const Node3D n2(Eigen::Matrix<double, 3, 1>(1,2,1));
    const Node3D n3(Eigen::Matrix<double, 3, 1>(1,3,1));
    assert(true == (n1 == n2));
    assert(false == (n1 == n3));
  }

  { // Check hash equality
    const Node3D n1(Eigen::Matrix<double, 3, 1>(1,2,1));
    const Node3D n2(Eigen::Matrix<double, 3, 1>(1,2,1));
    const Node3D n3(Eigen::Matrix<double, 3, 1>(1.0001,2.0001,1));
    assert(true == (n1.Hash() == n2.Hash()));
    assert(false == (n1.Hash() == n3.Hash()));
  }
}

void test_DirectedEdge2D() {
  { // Construction
    const auto source = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(1,2));
    const auto sink = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(2,2));
    const DirectedEdge2D edge(source, sink);
    assert(true == (*edge.Source() == *source));
    assert(true == (*edge.Sink() == *sink));
  }
}

void test_Graph2D() {
  { // Test graph access
    const auto n1 = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(1,2));
    const auto n2 = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(2,2));
    const auto n3 = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(3,2));
    const auto n4 = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(2,5));
    const DirectedEdge2D edge1(n1,n2), edge2(n1,n3), edge3(n3,n4), edge4(n2,n4);
    const Graph2D graph({edge1, edge2, edge3, edge4});

    const auto n5 = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(1,2));
    const auto n6 = std::make_shared<Node2D>(Eigen::Matrix<double, 2, 1>(0,0));
    assert(2 == graph.Edges(n5).size());
    assert(0 == graph.Edges(n6).size());  
  }
}

int main(int argc, char** argv) {
  test_Node2D();
  test_Node3D();
  test_DirectedEdge2D();
  test_Graph2D();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
