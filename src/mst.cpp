#include "graph.hpp"
#include <fstream>

int main(int argc, char const *argv[]) {
  std::fstream data_file("data/mst_data");
  mylib::Graph<int> graph(data_file);
  std::cout << graph << std::endl;
  auto mst = graph.mst_prim(1);
  std::cout <<  mst.first.size() << " edges in mst" << std::endl;
  double cost;
  for (auto item: mst.first) {
      std::cout << item << std::endl;
      cost += item.get_value();
  }
  std::cout << std::endl << cost << "(cost) should be equal to " << mst.second<< std::endl;
  return 0;
}
