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
#include "node2d.h"

namespace mediation_layer {
  namespace {
    template <class T>
    struct NodeWrapper {
      const std::shared_ptr<NodeWrapper> parent_;
      const std::shared_ptr<T> node_;
      const double cost_;
      const double heuristic_;
  
      NodeWrapper(const std::shared_ptr<NodeWrapper>& parent = nullptr,
                  const std::shared_ptr<T> node = nullptr,
                  const double cost = std::numeric_limits<double>::max(),
                  const double heuristic = std::numeric_limits<double>::max()) 
        : parent_(parent),
          node_(node),
          cost_(cost),
          heuristic_(heuristic)
      {};

      bool operator==(const NodeWrapper& rhs) const {
        return *this->node_ == *rhs.node_;
      }

      struct EqualsSharedPtr { 
        bool operator()(const std::shared_ptr<NodeWrapper>& lhs, 
                        const std::shared_ptr<NodeWrapper>& rhs) const {
          return *lhs == *rhs;
        }
      };
  
      bool operator<(const NodeWrapper& rhs) const {
       return this->cost_ + this->heuristic_ < rhs.cost_ + rhs.heuristic_; 
      };

      struct LessSharedPtr {
        bool operator()(const std::shared_ptr<NodeWrapper>& lhs, 
                       const std::shared_ptr<NodeWrapper>& rhs) const {
          return *lhs < *rhs;
        }
      };
  
      bool operator>(const NodeWrapper& rhs) const {
       return this->cost_ + this->heuristic_ > rhs.cost_ + rhs.heuristic_; 
      };

      struct GreaterSharedPtr {
        bool operator()(const std::shared_ptr<NodeWrapper>& lhs, 
                       const std::shared_ptr<NodeWrapper>& rhs) const {
          return *lhs > *rhs;
        }
      };

      struct Hash {
        size_t operator()(const NodeWrapper& dijkstra_node) const {
          return typename T::HashPointer()(dijkstra_node.node_);
        }
      };

      struct HashSharedPtr {
        size_t operator()(const std::shared_ptr<NodeWrapper>& dijkstra_node_ptr) const {
          return Hash()(*dijkstra_node_ptr);
        }
      };
    };
  }

  template <class T>
  struct AStar {
    private:
      std::function<double(const T&, const T&)> heuristic_;

    public:
      // Return value structure
      struct Path {
        struct Statistics {
          size_t num_nodes_explored{0};
          size_t path_length{0};
          std::chrono::duration<double> run_time{0};
 
          void Print() const {
            std::cout << "A* Statistics" << std::endl;
            std::cout << "\t" << "Number of Nodes Explored: " << num_nodes_explored << std::endl;
            std::cout << "\t" << "Number of Nodes in Path: " << path_length << std::endl;
            std::cout << "\t" << "Run Time: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(run_time).count() 
              << " ms" << std::endl;
            std::cout << std::endl;
          }
        };
       
        std::vector<std::shared_ptr<T>> nodes;
        Statistics statistics; 
      };

      AStar(const std::function<double(const T&, const T&)>& heuristic)
        : heuristic_(heuristic) {}

      Path Run(
          const Graph<T>& graph,
          const std::shared_ptr<T> start, 
          const std::shared_ptr<T> end) {
        Timer timer;
        timer.Start();
      
        // Paths from the start with a score equal 
        // to the total distance travelled
        std::priority_queue<
          std::shared_ptr<NodeWrapper<T>>, 
          std::vector<std::shared_ptr<NodeWrapper<T>>>, 
          class NodeWrapper<T>::GreaterSharedPtr> paths_to_explore;

        std::unordered_map<
          std::shared_ptr<NodeWrapper<T>>, 
          bool, 
          class NodeWrapper<T>::HashSharedPtr, 
          class NodeWrapper<T>::EqualsSharedPtr> explored_paths;

        // Add starting node
        paths_to_explore.push(std::make_shared<NodeWrapper<T>>(nullptr, start, 0.0, this->heuristic_(*start, *end)));
      
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
          const std::shared_ptr<NodeWrapper<T>> path_to_explore = paths_to_explore.top();
          paths_to_explore.pop();

          // If it's already been explored, the previous path is shorter. Skip.
          if(explored_paths.find(path_to_explore) != explored_paths.end()) {
            continue;
          }

          explored_paths[path_to_explore] = true;
          const std::shared_ptr<T> explored_node = path_to_explore->node_;
      
          // Check terminal conditions
          if(*explored_node == *end) {
            std::vector<std::shared_ptr<T>> solution;
            std::shared_ptr<NodeWrapper<T>> dijkstra_node_ptr = path_to_explore;
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
                  = std::make_shared<NodeWrapper<T>>(
                      path_to_explore, 
                      edge.sink_, 
                      path_to_explore->cost_ + edge.cost_,
                      this->heuristic_(*edge.sink_, *end));

                // If the path has not yet been explore, add it
                if(explored_paths.find(new_path_to_explore) == explored_paths.end()) {
                  paths_to_explore.push(new_path_to_explore);
                }
              });
        }  
      }
  };
}
