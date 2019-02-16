// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "a_star.h"
#include "node_eigen.h"

namespace mediation_layer {
  class AStar3D : public AStar<Node3D> {
    private:
      double Heuristic(const Node<Node3D>& a, 
                       const Node<Node3D>& b) { 
        return (a.Data() - b.Data()).norm(); 
      };

    public:
      AStar3D() : AStar(
          std::bind(
              &AStar3D::Heuristic, 
              this, 
              std::placeholders::_1, 
              std::placeholders::_2)) {}
  };
}
