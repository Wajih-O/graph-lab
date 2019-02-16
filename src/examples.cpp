/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 */
#include "edge.hpp"
#include "graph.hpp"
#include "list.hpp"
#include <iostream>
#include <string>

void list_lab() {
  mylib::List<int> *l = new mylib::List<int>(2);
  std::cout << *l->prepend(3)->prepend(4)->prepend(5) << std::endl;

  mylib::List<double> *double_list = new mylib::List<double>(2);
  std::cout << *double_list->prepend(7)->prepend(8.4)->prepend(5) << std::endl;
  std::cout << *l->get_tail() << std::endl;
  std::cout << *l << std::endl;
}

int graph_lab() {
  std::vector<int> nodes = {1, 2, 3, 4, 5};
  mylib::Graph<int> *graph =
      new mylib::Graph<int>(nodes, mylib::Edge<int>::generate(1., 10.));
  std::cout << *graph << std::endl;
  std::cout << *graph->filter(mylib::Edge<int>::more_than_filter(5.))
            << std::endl;
  std::cout << *graph->filter(mylib::Edge<int>::less_than_filter(5.))
            << std::endl;
}


/**
 *  Example with Lina
 */
 void lina_graph() {
   mylib::Graph<std::string> graph;
   graph.add_symmetric_edge(mylib::Edge<std::string>("A", "B", 2.0));
   graph.add_symmetric_edge(mylib::Edge<std::string>("B", "C", 3.0));
   graph.add_symmetric_edge(mylib::Edge<std::string>("B", "D", 4.0));

     std::cout << &graph << std::endl;
 }


/**
 * Wikipedia Dijkstra example
 */

void graph_example() {
  mylib::Graph<int> *graph = new mylib::Graph<int>();

  // Fill the graph https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

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

  // std::cout << *graph << std::endl;
  // std::cout << "filter less than 10:" << std::endl
  //           << *graph->filter(mylib::Edge<int>::less_than_filter(10));


  // check path length
  // std::cout << graph->get_path_length({1, 2, 4, 5}) << std::endl;
  auto shortest_path = graph->dijkstra(1, 5);
  for (auto item = shortest_path.begin(); item < shortest_path.end() -1; item++ ) {
    std::cout << *item << "-->" ;
  }
  std::cout << *(shortest_path.end()-1) << std::endl;

}
