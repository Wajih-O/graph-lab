# Graph-lab

A C++ graph algorithms lab. Implemented algorithms are:

## Dijkstra

 An algorithm conceived by computer scientist Edsger W. Dijkstra in 1956 for finding the shortest paths between nodes in a graph check https://en.wikipedia.org/wiki/Dijkstra's_algorithm for more details. The dijkstra example program uses the wikipedia graph example as a demo.

## MST

A minimum spanning tree (MST) or minimum weight spanning tree is a subset of the edges of a connected,edge-weighted undirected graph that connects all the vertices together, without any cycles and with the minimum possible total edge weight (src wikipedia https://en.wikipedia.org/wiki/Minimum_spanning_tree)

The variant implemented here is Prim's algorithm https://en.wikipedia.org/wiki/Prim%27s_algorithm.

The Kruskal's version will be added soon :)

## Building

Building is based on CMake and the hunter packages manager.

To build all targets dijkstra (example) mst (example) and tests run `make`

To build targets individually run `make [dijkstra | mst | graph_tests]`