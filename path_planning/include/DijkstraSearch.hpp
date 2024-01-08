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

    bool DijkstraSearch::is_same_node(double node_x, double node_y, Node *node_b)
    {
        double dist = std::hypot(node_x - node_b->x, node_y - node_b->y);
        return dist <= 0.1;
    }

    bool DijkstraSearch::is_same_node(Node *node_a, Node *node_b)
    {
        // TODO: check the logic of this function
        return node_a == node_b;
    }

    int DijkstraSearch::find_id(Node *target_node)
    {
        for (size_t i = 0; i < node_x.size(); i++)
        {
            if (is_same_node(node_x[i], node_y[i], target_node))
            {
                return i;
            }
        }

        return -1;
    }

    bool DijkstraSearch::has_node_in_set(tsl::ordered_map<int, Node *> &target_set, Node *node)
    {
        for (const auto &it : target_set)
        {
            if (is_same_node(it.second, node))
            {
                return true;
            }
        }
        return false;
    }

    std::vector<std::vector<double>> DijkstraSearch::generate_final_path(tsl::ordered_map<int, Node *> &close_set, Node *goal_node)
    {
        std::vector<std::vector<double>> path{{goal_node->x, goal_node->y}};
        int parent = goal_node->parent;

        while (parent != -1)
        {
            Node *node = close_set[parent];
            path.push_back({node->x, node->y});
            parent = node->parent;
        }

        return path;
    }

    std::vector<std::vector<double>> DijkstraSearch::search()
    {
        Node *start = new Node(s_x, s_y);
        Node *goal = new Node(g_x, g_y);

        Node *current_node = NULL;

        tsl::ordered_map<int, Node *> open_set;
        tsl::ordered_map<int, Node *> close_set;

        open_set[find_id(start)] = start;

        while (true)
        {
            if (has_node_in_set(close_set, goal))
            {
                std::cout << "GOAL is found!" << std::endl;
                goal->parent = current_node->parent;
                goal->cost = current_node->cost;
                break;
            }
            else if (open_set.size() == 0)
            {
                std::cout << ":X cannot find valid path" << std::endl;
                break;
            }

            int current_id; // TODO: implement
            current_node = open_set[current_id];

            open_set.erase(current_id);

            close_set[current_id] = current_node;

            for (size_t i = 0; i < edge_ids_list[current_id].size(); i++)
            {
                int n_id = edge_ids_list[current_id][i];
                double dx = node_x[n_id] - current_node->x;
                double dy = node_y[n_id] - current_node->y;

                double dist = std::hypot(dx, dy);
                Node *node = new Node(node_x[n_id], node_y[n_id], current_node->cost + dist, current_id);

                if (close_set.find(n_id) != close_set.end())
                {
                    continue;
                }
                if (open_set.find(n_id) != open_set.end())
                {
                    if (open_set[n_id]->cost > node->cost)
                    {
                        open_set[n_id] = node;
                    }
                }
                else
                {
                    open_set[n_id] = node;
                }
            }
        }
        std::vector<std::vector<double>> path = generate_final_path(close_set, goal);

        return path;
    }

}

#endif