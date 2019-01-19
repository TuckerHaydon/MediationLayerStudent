// Author: Tucker Haydon

#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

#include "node.h"
#include "directed_edge.h"
#include "graph.h"
#include "dijkstra.h"


/**   Simple Graph
 *     b - 3 - d
 *    /         \
 *   1           ?
 *  /             \
 * a               f - 1 - g
 *  \             /
 *   2           1
 *    \         /
 *     c - 2 - e
 */

int main(int argc, char** argv) {

  Node a("a"), b("b"), c("c"), d("d"), e("e"), f("f"), g("g");

  DirectedEdge e1(&a, &b, 1), 
               e2(&a, &c, 2), 
               e3(&b, &d, 3), 
               e4(&c, &e, 2), 
               e5(&d, &f, 0.5), 
               e6(&e, &f, 1),
               e7(&f, &g, 1);

  Graph graph;
  graph.AddEdges({&e1, &e2, &e3, &e4, &e5, &e6, &e7});

  Dijkstra dijkstra(&graph);
  std::vector<Node*> path = dijkstra.Run(&a, &g);

  std::cout << "Path:" << std::endl;
  std::for_each(
      path.begin(),
      path.end(),
      [](Node* node){
        std::cout << node->Id() << std::endl;
      });

  return 0;
}
