#ifndef DIJKSTRA_SEARCH_HPP
#define DIJKSTRA_SEARCH_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <random>
#include <limits>
#include <algorithm>
#include "tsl/ordered_map.h"

namespace dijkstra
{
    class Node
    {
    public:
        double x;
        double y;
        double cost;
        int parent;

        Node(double x_, double y_, double cost_ = 0.0, int parent_ = -1) : x(x_), y(y_), cost(cost_), parent(parent_){};
        Node() : x(0.0), y(0.0), cost(0.0), parent(-1){};
    };

    class DijkstraSearch
    {
    public:
        double s_x;
        double s_y;
        double g_x;
        double g_y;
        std::vector<double> node_x;
        std::vector<double> node_y;
        std::vector<std::vector<int>> edge_ids_list;

        DijkstraSearch(double s_x_, double s_y_, double g_x_, double g_y_, std::vector<double> node_x_, std::vector<double> node_y_, std::vector<std::vector<int>> edge_ids_list_) : s_x(s_x_), s_y(s_y_), g_x(g_x_), g_y(g_y_), node_x(node_x_), node_y(node_y_), edge_ids_list(edge_ids_list_){};
        bool is_same_node(double node_x, double node_y, Node *node_b);
        bool is_same_node(Node *node_a, Node *node_b);
        int find_id(Node *target_node);
        bool has_node_in_set(tsl::ordered_map<int, Node *> &target_set, Node *node);
        std::vector<std::vector<double>> generate_final_path(tsl::ordered_map<int, Node *> &close_set, Node *goal_node);
        std::vector<std::vector<double>> search();
    };

}

#endif