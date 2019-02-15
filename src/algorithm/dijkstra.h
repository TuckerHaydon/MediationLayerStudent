// Author: Tucker Haydon

#pragma once

#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <limits>
#include <memory>
#include <chrono>

#include "graph.h"
#include "timer.h"

namespace mediation_layer {
  namespace {
    template <class T>
    struct DijkstraNode {
      const std::shared_ptr<DijkstraNode> parent_;
      const std::shared_ptr<Node<T>> node_;
      const double cost_;
  
      DijkstraNode(const std::shared_ptr<DijkstraNode>& parent = nullptr,
                  const std::shared_ptr<Node<T>> node = nullptr,
                  const double cost = std::numeric_limits<double>::max()) 
        : parent_(parent),
          node_(node),
          cost_(cost)
      {};

      bool operator==(const DijkstraNode& rhs) const {
        return *this->node_ == *rhs.node_;
      }

      struct EqualsSharedPtr { 
        bool operator()(const std::shared_ptr<DijkstraNode>& lhs, 
                        const std::shared_ptr<DijkstraNode>& rhs) const {
          return *lhs == *rhs;
        }
      };
  
      bool operator<(const DijkstraNode& rhs) const {
       return this->cost_ < rhs.cost_; 
      };

      struct LessSharedPtr {
        bool operator()(const std::shared_ptr<DijkstraNode>& lhs, 
                       const std::shared_ptr<DijkstraNode>& rhs) const {
          return *lhs < *rhs;
        }
      };
  
      bool operator>(const DijkstraNode& rhs) const {
       return this->cost_ > rhs.cost_; 
      };

      struct GreaterSharedPtr {
        bool operator()(const std::shared_ptr<DijkstraNode>& lhs, 
                       const std::shared_ptr<DijkstraNode>& rhs) const {
          return *lhs > *rhs;
        }
      };

      struct Hash {
        size_t operator()(const DijkstraNode& dijkstra_node) const {
          return typename Node<T>::HashPointer()(dijkstra_node.node_);
        }
      };

      struct HashSharedPtr {
        size_t operator()(const std::shared_ptr<DijkstraNode>& dijkstra_node_ptr) const {
          return Hash()(*dijkstra_node_ptr);
        }
      };
    };
  }

  template <class T>
  struct Dijkstra {

    // Return value structure
    struct Path {
      struct Statistics {
        size_t num_nodes_explored{0};
        size_t path_length{0};
        std::chrono::duration<double> run_time{0};
 
        void Print() const {
          std::cout << "Dijkstra Statistics" << std::endl;
          std::cout << "\t" << "Number of Nodes Explored: " << num_nodes_explored << std::endl;
          std::cout << "\t" << "Number of Nodes in Path: " << path_length << std::endl;
          std::cout << "\t" << "Run Time: " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(run_time).count() 
            << " ms" << std::endl;
          std::cout << std::endl;
        }
      };
     
      std::vector<std::shared_ptr<Node<T>>> nodes;
      Statistics statistics; 
    };

    // std::vector<std::shared_ptr<Node<T>>> Run(
    Path Run(
        const Graph<T>& graph, 
        const std::shared_ptr<Node<T>> start, 
        const std::shared_ptr<Node<T>> end) {
      Timer timer;
      timer.Start();
    
      // Paths from the start with a score equal 
      // to the total distance travelled
      std::priority_queue<
        std::shared_ptr<DijkstraNode<T>>, 
        std::vector<std::shared_ptr<DijkstraNode<T>>>, 
        class DijkstraNode<T>::GreaterSharedPtr> paths_to_explore;

      std::unordered_map<
        std::shared_ptr<DijkstraNode<T>>, 
        bool, 
        class DijkstraNode<T>::HashSharedPtr, 
        class DijkstraNode<T>::EqualsSharedPtr> explored_paths;

      // Add starting node
      paths_to_explore.push(std::make_shared<DijkstraNode<T>>(nullptr, start, 0.0));
    
      while(true) {
        // If no solution found, return empty vector
        if(paths_to_explore.empty()) {
          auto duration = timer.Stop();
          Path path;
          path.nodes = {};
          path.statistics.num_nodes_explored = explored_paths.size();
          path.statistics.path_length = 0;
          path.statistics.run_time = duration;
          return path;
        }

    
        // Get next node
        const std::shared_ptr<DijkstraNode<T>> path_to_explore = paths_to_explore.top();
        paths_to_explore.pop();

        // If it's already been explored, the previous path is shorter. Skip.
        if(explored_paths.find(path_to_explore) != explored_paths.end()) {
          continue;
        }

        explored_paths[path_to_explore] = true;
        const std::shared_ptr<Node<T>> explored_node = path_to_explore->node_;
    
        // Check terminal conditions
        if(*explored_node == *end) {
          std::vector<std::shared_ptr<Node<T>>> solution;
          std::shared_ptr<DijkstraNode<T>> dijkstra_node_ptr = path_to_explore;
          while(dijkstra_node_ptr->parent_ != nullptr) {
            solution.push_back(dijkstra_node_ptr->node_);
            dijkstra_node_ptr = dijkstra_node_ptr->parent_;
          }
          solution.push_back(start);
    
          // Return
          auto duration = timer.Stop();
          Path path;
          path.nodes = solution;
          path.statistics.num_nodes_explored = explored_paths.size();
          path.statistics.path_length = solution.size();
          path.statistics.run_time = duration;
          return path;
        }
    
        // Expand the path 
        const std::vector<DirectedEdge<T>>& edges = graph.Edges(explored_node);
        std::for_each(
            edges.begin(),
            edges.end(),
            [&](const DirectedEdge<T>& edge) mutable {
              auto new_path_to_explore 
                = std::make_shared<DijkstraNode<T>>(
                    path_to_explore, 
                    edge.sink_, 
                    path_to_explore->cost_ + edge.cost_);

              // If the path has not yet been explore, add it
              if(explored_paths.find(new_path_to_explore) == explored_paths.end()) {
                paths_to_explore.push(new_path_to_explore);
              }
            });
      }  
    }
  };

  using Dijkstra2D = Dijkstra<Eigen::Matrix<double, 2, 1>>;
  using Dijkstra3D = Dijkstra<Eigen::Matrix<double, 3, 1>>;
}
