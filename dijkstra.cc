// Author: Tucker Haydon

#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <iostream>

#include "dijkstra.h"

namespace{
  typedef std::vector<const Node*> path_t;

  struct Path {
    std::vector<const DirectedEdge*> path_;
    double score_;

    Path(const std::vector<const DirectedEdge*>& path, double score) 
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

path_t Dijkstra::Run(const Node* start, const Node* end) const {
  TicToc();

  // Paths from the start with a score equal 
  // to the total distance travelled
  std::priority_queue<Path, std::vector<Path>, std::greater<Path>> paths;

  // Expand starting node
  const std::vector<const DirectedEdge*>& edges = this->graph_->GetEdges(start);
  std::for_each(
      edges.begin(),
      edges.end(),
      [&](const DirectedEdge* edge) mutable {
        std::vector<const DirectedEdge*> path = {edge};
        paths.emplace(path, edge->Cost());
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
    const Node* n = path_to_explore.path_.back()->Sink();

    // Check terminal conditions
    if(*n == *end) {
      path_t solution;
      std::for_each(
          path_to_explore.path_.begin(),
          path_to_explore.path_.end(),
          [&](const DirectedEdge* edge) mutable {
            solution.push_back(edge->Source());
          });
      solution.push_back(end);

      TicToc();
      return solution;
    }

    // Expand the path 
    const std::vector<const DirectedEdge*>& edges = this->graph_->GetEdges(n);
    std::for_each(
        edges.begin(),
        edges.end(),
        [&](const DirectedEdge* edge) mutable {

          // Copy old path and push new edge onto it
          std::vector<const DirectedEdge*> edges_new = path_to_explore.path_;
          edges_new.push_back(edge);

          double score_new = path_to_explore.score_ + edge->Cost();

          paths.emplace(edges_new, score_new);
        });
  }  
}
