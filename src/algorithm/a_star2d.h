// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "a_star.h"
#include "node.h"

namespace mediation_layer {
  class AStar2D : public AStar<Eigen::Vector2d> {
    private:
      double Heuristic(const Node<Eigen::Vector2d>& a, 
                       const Node<Eigen::Vector2d>& b) { 
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
