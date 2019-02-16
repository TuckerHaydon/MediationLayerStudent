// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "a_star.h"
#include "node_eigen.h"

namespace mediation_layer {
  class AStar2D : public AStar<Node2D> {
    private:
      double Heuristic(const Node2D& a, 
                       const Node2D& b) { 
        return (a.Data() - b.Data()).norm(); 
      };

    public:
      AStar2D() : AStar(
          std::bind(
              &AStar2D::Heuristic, 
              this, 
              std::placeholders::_1, 
              std::placeholders::_2)) {}
  };
}
