/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 */

#include <iostream>
#include <string>
#include <numeric>
#include <exception>

#include "edge.hpp"
#include "graph.hpp"


/**
 * Graph example from wikipedia/disjkstra
 * https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
 */

void graph_example_wikipedia() {
  mylib::Graph<int> *graph = new mylib::Graph<int>();

  graph->add_symmetric_edge(mylib::Edge<int>(1, 2, 7.));
  graph->add_symmetric_edge(mylib::Edge<int>(1, 3, 9.));
  graph->add_symmetric_edge(mylib::Edge<int>(1, 6, 14.));

  graph->add_symmetric_edge(mylib::Edge<int>(2, 1, 7.));
  graph->add_symmetric_edge(mylib::Edge<int>(2, 3, 10.));
  graph->add_symmetric_edge(mylib::Edge<int>(2, 4, 15.));

  graph->add_symmetric_edge(mylib::Edge<int>(3, 6, 2.));
  graph->add_symmetric_edge(mylib::Edge<int>(3, 4, 6.));

  graph->add_symmetric_edge(mylib::Edge<int>(4, 5, 6.));
  graph->add_symmetric_edge(mylib::Edge<int>(5, 6, 9.));

  std::cout << " Wikipedia dijkstra example shortest path from 1 -> 5" << std::endl<< "    Shortest path: ";
  auto shortest_path = graph->dijkstra(1, 5);
  for (auto item = shortest_path.begin(); item < shortest_path.end() -1; item++ ) {
    std::cout << *item << "-->" ;
  }
  std::cout << *(shortest_path.end()-1) << std::endl;

  for (auto node: graph->mst_prim(1)) {
    std::cout << "mst: "<< node << std::endl;
  }
for (auto node: graph->mst_prim(2)) {
    std::cout << "mst: "<< node << std::endl;
  }

  delete(graph);
}


/**
 *  string graph example
 */
void string_graph() {

  mylib::Graph<std::string> *graph = new mylib::Graph<std::string>();
  graph->add_symmetric_edge(
      mylib::Edge<std::string>("node_0", "node_3", 50.));
  graph->add_symmetric_edge(
      mylib::Edge<std::string>("node_3", "n_5", 17.));
  graph->add_symmetric_edge(mylib::Edge<std::string>("n_5", "n_2", 3.));
  graph->add_symmetric_edge(
      mylib::Edge<std::string>("node_3", "node_0", 9.));

  // check path length
  std::cout << graph->get_path_length({"node_0", "node_3", "n_5", "n_2"})
            << std::endl;

  auto reachable_from_node_3 = graph->neighbors("node_3");
  std::cout << *graph << std::endl;
  delete(graph);
}

/**
 * Averate 1->n path length for a random graph
 * @param density: graph density
 * @nodes_numver: vertices number in the graph
 */

double average_path_from_the_first_node(double density=.1, int nodes_number=50) {
  if (nodes_number > 1) {
  std::vector<int> nodes(nodes_number);
  std::iota(nodes.begin(), nodes.end(), 0);

  // build a random graph between nodes with density equal to param density  and values between 1. and 10.
  mylib::Graph<int> graph(nodes, mylib::Edge<int>::generate(1., 10.), density);
  double sum_ = 0;
  int valid_paths = 0;
  for (auto node=nodes.begin() + 1; node != nodes.end(); node++) {
    auto shortest_path = graph.dijkstra(*nodes.begin(), *node);
    auto path_length = graph.get_path_length(shortest_path);
    if (path_length != -1) {
      sum_ += path_length;
      valid_paths += 1;
    }
  }

  if (valid_paths){
    return sum_/valid_paths;
  }
  return -1;
  // std::cout <<  graph.get_path_length(shortest_path)<< std::endl;
  } else {
    throw "nodes number should be strictly greater than one";
  }

}


int main() {
  graph_example_wikipedia();
  std::vector<double> densities = {.2, .4, 1.};
  //  equivalent to density 20% and 40% adn 100% ( average path 1-> n should be statistically lower with hight density)
  for (auto density: densities) {
    std::cout <<  "density: " << density << ", average path 1->n length: "<<average_path_from_the_first_node(density) << std::endl;
  }
}