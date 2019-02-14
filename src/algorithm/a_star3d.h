// Author: Tucker Haydon

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "a_star.h"
#include "node.h"

namespace path_planning {
  class AStar3D : public AStar<Eigen::Vector3d> {
    private:
      double Heuristic(const Node<Eigen::Vector3d>& a, 
                       const Node<Eigen::Vector3d>& b) { 
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
