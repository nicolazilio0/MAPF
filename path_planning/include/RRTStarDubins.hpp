#ifndef RRT_STAR_DUBINS_HPP
#define RRT_STAR_DUBINS_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <random>
#include <limits>
#include <algorithm>

#include "DubinsPath.hpp"

namespace rrtstar
{

    class Node
    {
    public:
        double x;
        double y;
        double yaw;
        std::vector<double> path_x;
        std::vector<double> path_y;
        std::vector<double> path_yaw;
        Node *parent;
        double cost;

        Node(double x, double y, double yaw) : x(x), y(y), yaw(yaw), path_x(), path_y(), path_yaw(), parent(NULL), cost(0.0){};
        Node() : x(0), y(0), yaw(0), path_x(), path_y(), path_yaw(), parent(NULL), cost(0.0){};
    };

    class RRTStarDubins
    {
    public:
        Node *start;
        Node *end;
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
        DubinsPath dubbins_planner;
        std::vector<Node *> node_list;

        std::random_device rd;
        std::mt19937 generator{rd()};

        RRTStarDubins(Node *start, Node *goal, std::vector<std::vector<double>> obstacle_list,
                      double min_rand, double max_rand, int goal_sample_rate = 10, int max_iter = 500,
                      double connect_circle_dist = 50.0, double robot_radius = 0.353);

        Node *get_random_node();
        Node *get_nearest_node(Node *rnd_node);
        Node *steer(Node *from_node, Node *to_node);
        bool check_collision(Node *node);
        std::vector<Node *> find_near_nodes(Node *new_node);
        double calc_new_cost(Node *from_node, Node *to_node);
        Node *choose_parent(Node *new_node, std::vector<Node *> nnodes);
        void propagate_cost_to_leaves(Node *parent_node);
        void rewire(Node *new_node, std::vector<Node *> nnodes);
        double calc_dist_to_goal(double x, double y);
        Node *search_best_goal_node();
        std::vector<std::vector<double>> generate_final_course(Node *goal_node);
        std::vector<std::vector<double>> planning(bool search_until_max_iter = true);
    };

}
#endif