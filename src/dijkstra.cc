// Author: Tucker Haydon

#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <limits>

#include "dijkstra.h"

namespace path_planning {
  namespace {
    struct DijkstraNode {
      const std::shared_ptr<DijkstraNode> parent_;
      const Node node_;
      const double cost_;

      static std::shared_ptr<DijkstraNode> NULL_NODE_PTR;
  
      DijkstraNode(const std::shared_ptr<DijkstraNode>& parent = NULL_NODE_PTR,
                  const Node& node = Node::NULL_NODE,
                  const double cost = std::numeric_limits<double>::max()) 
        : parent_(parent),
          node_(node),
          cost_(cost)
      {};

      bool operator==(const DijkstraNode& rhs) const {
        return this->node_ == rhs.node_;
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
          return Node::Hash()(dijkstra_node.node_);
        }
      };

      struct HashSharedPtr {
        size_t operator()(const std::shared_ptr<DijkstraNode>& dijkstra_node_ptr) const {
          return Hash()(*dijkstra_node_ptr);
        }
      };
    };

    std::shared_ptr<DijkstraNode> DijkstraNode::NULL_NODE_PTR = nullptr;
  
    void TicToc() {
      static bool tic{false};
  
      typedef std::chrono::high_resolution_clock Time;
      typedef std::chrono::milliseconds ms;
      typedef std::chrono::duration<double> duration_t;
  
      static std::chrono::time_point<Time> start, end; 
  
      if(!tic) {
        start = Time::now();
        tic = true;
      } else {
        end = Time::now();
        duration_t elapsed_time = end - start;
        ms elapsed_ms = std::chrono::duration_cast<ms>(elapsed_time);
        std::cout << "Dijkstra finished in " << elapsed_ms.count() << " ms" << std::endl;
        tic = false;
      }
    }
  }

  std::vector<Node> Dijkstra::Run(const Node& start, const Node& end) const {
    TicToc();
  
    // Paths from the start with a score equal 
    // to the total distance travelled
    std::priority_queue<std::shared_ptr<DijkstraNode>, std::vector<std::shared_ptr<DijkstraNode>>, DijkstraNode::GreaterSharedPtr> paths_to_explore;
    std::unordered_map<std::shared_ptr<DijkstraNode>, bool, DijkstraNode::HashSharedPtr, DijkstraNode::EqualsSharedPtr> explored_paths;

    // Add starting node
    paths_to_explore.push(std::make_shared<DijkstraNode>(DijkstraNode::NULL_NODE_PTR, start, 0.0));
  
    while(true) {
      // If no solution found, return empty vector
      if(paths_to_explore.empty()) {
        TicToc();
        return {};
      }

  
      // Get next node
      const std::shared_ptr<DijkstraNode> path_to_explore = paths_to_explore.top();
      paths_to_explore.pop();

      // If it's already been explored, the previous path is shorter. Skip.
      if(explored_paths.find(path_to_explore) != explored_paths.end()) {
        continue;
      }

      explored_paths[path_to_explore] = true;
      const Node& explored_node = path_to_explore->node_;
  
      // Check terminal conditions
      if(explored_node == end) {
        std::vector<Node> solution;
        std::shared_ptr<DijkstraNode> dijkstra_node_ptr = path_to_explore;
        while(dijkstra_node_ptr->parent_ != DijkstraNode::NULL_NODE_PTR) {
          solution.push_back(dijkstra_node_ptr->node_);
          dijkstra_node_ptr = dijkstra_node_ptr->parent_;
        }
  
        std::cout << "Num nodes explored: " << explored_paths.size() << std::endl;

        TicToc();
        return solution;
      }
  
      // Expand the path 
      const std::vector<DirectedEdge>& edges = this->graph_->Edges(explored_node);
      std::for_each(
          edges.begin(),
          edges.end(),
          [&](const DirectedEdge& edge) mutable {
            std::shared_ptr<DijkstraNode> new_path_to_explore = std::make_shared<DijkstraNode>(path_to_explore, edge.sink_, path_to_explore->cost_ + edge.cost_);

            // If the path has not yet been explore, add it
            if(explored_paths.find(new_path_to_explore) == explored_paths.end()) {
              paths_to_explore.push(new_path_to_explore);
            }
          });
    }  
  }
}
