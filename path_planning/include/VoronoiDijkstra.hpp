#ifndef VORONOI_DIJKSTRA_HPP
#define VORONOI_DIJKSTRA_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <random>
#include <limits>
#include <algorithm>

#include "kdtree.hpp"
#include "DijkstraSearch.hpp"

class VoronoiDijkstra
{
public:
    double robot_radius;
    std::vector<double> sample_x;
    std::vector<double> sample_y;

    const double N_KNN = 30;
    const double MAX_EDGE_LEN = 30.0;

    // TODO: add Dijkstra search
    VoronoiDijkstra(std::vector<double> voronoi_x, std::vector<double> voronoi_y, double robot_radius_ = 0.5);

    bool check_collision(double s_x, double s_y, double g_x, double g_y, Kdtree::KdTree *obstalces_tree);
    std::vector<std::vector<int>> generate_roadmap_info(Kdtree::KdTree *obstalces_tree);
    std::vector<std::vector<double>> planning(double s_x, double s_y, double g_x, double g_y, std::vector<double> o_x, std::vector<double> o_y);
};

#endif