#ifndef RRT_STAR_DUBINS_HPP
#define RRT_STAR_DUBINS_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <random>
#include <limits>

#include "DubinsPath.hpp"

class Node
{
public:
    double x;
    double y;
    double yaw;
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;
    const Node *parent;
    double cost;

    Node(double x, double y, double yaw) : x(x), y(y), yaw(yaw), parent(nullptr){};
};

class RRTStarDubins
{
public:
    Node start;
    Node end;
    double min_rand;
    double max_rand;
    int goal_sample_rate;
    int max_iter;
    std::vector<std::vector<double>> obstacle_list;
    double connect_circle_dist;
    double curvature;
    double goal_yaw_th;
    double goal_xy_th;
    double robot_radius;
    mutable DubinsPath dubbins_planner;
    std::vector<Node> node_list;

    RRTStarDubins(std::vector<double> start, std::vector<double> goal, std::vector<std::vector<double>> obstacle_list,
                  double min_rand, double max_rand, int goal_sample_rate = 10, int max_iter = 100,
                  double connect_circle_dist = 50.0, double robot_radius = 0.0);

    Node get_random_node();
    size_t get_nearest_node_index(const std::vector<Node> &node_list, const Node &rnd_node) const;
    Node *steer(const Node &from_node, const Node &to_node);
    bool check_collision(const Node *node, const std::vector<std::vector<double>> &obstacle_list, double robot_radius) const;
    std::vector<size_t> find_near_nodes(const Node &new_node) const;
    double calc_new_cost(const Node &from_node, const Node &to_node) const;
    Node *choose_parent(const Node &new_node, const std::vector<size_t> &near_inds);
    void propagate_cost_to_leaves(Node *parent_node);
    void rewire(const Node &new_node, const std::vector<size_t> &near_inds);
    double calc_dist_to_goal(double x, double y) const;
    int search_best_goal_node() const;
    std::vector<std::vector<double>> generate_final_course(int goal_index) const;
    std::vector<std::vector<double>> planning(bool search_until_max_iter = true);
};

#endif