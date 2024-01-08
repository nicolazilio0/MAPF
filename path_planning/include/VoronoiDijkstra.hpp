#ifndef VORONOI_DIJKSTRA_HPP
#define VORONOI_DIJKSTRA_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <random>
#include <limits>
#include <algorithm>
#include "../include/kdtree.hpp"

class VoronoiDijkstra
{
public:
    float robot_radius;
    std::vector<float> sample_x;
    std::vector<float> sample_y;

    const float N_KNN = 15;
    const float MAX_EDGE_LEN = 30.0;

    // TODO: add Dijkstra search
    VoronoiDijkstra(std::vector<float> voronoi_x, std::vector<float> voronoi_y, float robot_radius_);

    bool check_collision(float s_x, float s_y, float g_x, float g_y, Kdtree::KdTree *obstalces_tree);
    std::vector<std::vector<int>> generate_roadmap_info(Kdtree::KdTree *obstalces_tree);
    std::vector<std::vector<float>> planning(float s_x, float s_y, float g_x, float g_y, std::vector<float> o_x, std::vector<float> o_y);
};

#endif