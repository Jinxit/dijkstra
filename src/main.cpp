#include <iostream>
#include "map.hpp"


int main(int argc, const char* argv[])
{
    std::vector<dijkstra::point> graph;
    graph.emplace_back(0, 0);
    graph.emplace_back(1, 0);
    graph.emplace_back(0, 1);
    graph.emplace_back(1, 1);
    graph.emplace_back(2, 1);
    graph.emplace_back(2, 2);
    graph.emplace_back(3, 5);
    dijkstra::map m(std::move(graph));
    m.connect(0, 1);
    m.connect(0, 2);
    m.connect(1, 3);
    m.connect(2, 3);
    m.connect(3, 4);
    m.connect(4, 5);
    m.connect(5, 6);
    std::cout << "precompute" << std::endl;
    m.precompute({dijkstra::point(0.2, 0.1),
                  dijkstra::point(2, 2)});

    std::cout << "find" << std::endl;
    m.find(dijkstra::point(0, 0), dijkstra::point(2, 2));
    std::cout << "find" << std::endl;
    m.find(dijkstra::point(1, 1), dijkstra::point(2, 2));
    std::cout << "find" << std::endl;
    m.find(dijkstra::point(0, 0), dijkstra::point(3, 5));
}