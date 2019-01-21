// Author: Tucker Haydon

#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <sstream>

#include "a_star.h"

namespace pathing {
  namespace {
    struct Path {
      std::vector<DirectedEdge> path_;
      double score_;
  
      Path(const std::vector<DirectedEdge>& path, double score) 
        : path_(path),
          score_(score) 
      {};
  
      bool operator<(const Path& rhs) const {
       return this->score_ < rhs.score_; 
      }
  
      bool operator>(const Path& rhs) const {
       return this->score_ > rhs.score_; 
      }
    };

    double Heuristic(const Node& a, const Node& b) {
      std::string id_a = a.Id(); 
      std::string id_b = b.Id();

      id_a.erase(id_a.begin());
      id_a.erase(id_a.end()-1);
      id_b.erase(id_b.begin());
      id_b.erase(id_b.end()-1);

      double a_pos[2], b_pos[2];

      {
        std::istringstream id_a_ss(id_a);
        std::string s;
        size_t idx = 0;
    	  while (getline(id_a_ss, s, ',')) {
          a_pos[idx] = std::stod(s);
          ++idx;
    	  }
      }

      {
        std::istringstream id_b_ss(id_b);
        std::string s;
        size_t idx = 0;
    	  while (getline(id_b_ss, s, ',')) {
          b_pos[idx] = std::stod(s);
          ++idx;
    	  }
      }

      return std::sqrt( std::pow(a_pos[0] - b_pos[0],2) + std::pow(a_pos[1] - b_pos[1],2));
    }
  
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
        std::cout << "A* finished in " << elapsed_ms.count() << " ms" << std::endl;
        tic = false;
      }
    }
  }
  std::vector<Node> AStar::Run(const Node& start, const Node& end) const {
    TicToc();
  
    // Paths from the start with a score equal 
    // to the total distance travelled
    std::priority_queue<Path, std::vector<Path>, std::greater<Path>> paths;
    std::unordered_map<Node, bool, Node::Hash> visited_nodes;
 
    // Expand starting node
    const std::vector<DirectedEdge>& edges = this->graph_->Edges(start);
    std::for_each(
        edges.begin(),
        edges.end(),
        [&](const DirectedEdge& edge) mutable {
          std::vector<DirectedEdge> path = {edge};
          paths.emplace(path, edge.Cost() + Heuristic(edge.Sink(), end));
        });
  
    while(true) {
      // If no solution found, return empty vector
      if(paths.empty()) {
        TicToc();
        return {};
      }
  
      // Get next node
      Path path_to_explore = paths.top();
      paths.pop();
      const Node n = path_to_explore.path_.back().Sink();
      visited_nodes[n] = true;
  
      // Check terminal conditions
      if(n == end) {
        std::vector<Node> solution;
        std::for_each(
            path_to_explore.path_.begin(),
            path_to_explore.path_.end(),
            [&](const DirectedEdge& edge) mutable {
              solution.push_back(edge.Source());
            });
        solution.push_back(end);
  
        std::cout << "Num visited: " << visited_nodes.size() << std::endl;
        TicToc();
        return solution;
      }
  
      // Expand the path 
      const std::vector<DirectedEdge>& edges = this->graph_->Edges(n);
      std::for_each(
          edges.begin(),
          edges.end(),
          [&](const DirectedEdge& edge) mutable {
            // If the end of the edge has already been visited, pass
            if(visited_nodes.find(edge.Sink()) != visited_nodes.end()) {
              return;
            }
  
            // Copy old path and push new edge onto it
            std::vector<DirectedEdge> edges_new = path_to_explore.path_;
            edges_new.push_back(edge);
  
            double score_new = path_to_explore.score_ + edge.Cost() + Heuristic(edge.Sink(), end);
  
            paths.emplace(edges_new, score_new);
          });
    }  
  }
}
