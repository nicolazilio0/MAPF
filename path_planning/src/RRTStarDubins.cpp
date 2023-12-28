#include "../include/RRTStarDubins.hpp"

using namespace rrtstar;

RRTStarDubins::RRTStarDubins(Node *start, Node *goal, std::vector<std::vector<double>> obstacle_list,
                             double min_rand, double max_rand, int goal_sample_rate, int max_iter,
                             double connect_circle_dist, double robot_radius)
    : start(start),
      end(goal),
      min_rand(min_rand),
      max_rand(max_rand),
      goal_sample_rate(goal_sample_rate),
      max_iter(max_iter),
      obstacle_list(obstacle_list),
      connect_circle_dist(connect_circle_dist),
      curvature(0.75),
      goal_yaw_th(0.0174532),
      goal_xy_th(0.5),
      robot_radius(robot_radius),
      dubbins_planner()
{
}

Node *RRTStarDubins::get_random_node()
{

    std::uniform_int_distribution<int> int_distribution(0, 100);
    if (int_distribution(generator) > goal_sample_rate)
    {
        std::uniform_real_distribution<double> x_distribution(min_rand, max_rand);
        std::uniform_real_distribution<double> y_distribution(min_rand, max_rand);
        std::uniform_real_distribution<double> yaw_distribution(-M_PI, M_PI);

        double x = x_distribution(generator);
        double y = y_distribution(generator);
        double yaw = yaw_distribution(generator);

        Node *rnd = new Node(x, y, yaw);
        return rnd;
    }
    else
    {
        return end;
    }
}

// Assuming node_list is a vector<Node>
Node *RRTStarDubins::get_nearest_node(Node *rnd_node)
{
    double min_dist = std::numeric_limits<double>::max();
    Node *nearest_node;

    for (Node *node : node_list)
    {
        double distance = std::pow(node->x - rnd_node->x, 2) + std::pow(node->y - rnd_node->y, 2);
        if (distance < min_dist)
        {
            min_dist = distance;
            nearest_node = node;
        }
    }

    return nearest_node;
}

Node *RRTStarDubins::steer(Node *from_node, Node *to_node)
{
    auto [px, py, pyaw, mode, course_lengths] = dubbins_planner.plan_dubins_path(
        from_node->x, from_node->y, from_node->yaw,
        to_node->x, to_node->y, to_node->yaw, curvature);

    if (px.size() <= 1)
    { // cannot find a Dubins path
        return nullptr;
    }

    Node *new_node = new Node(px.back(), py.back(), pyaw.back());
    new_node->path_x = px;
    new_node->path_y = py;
    new_node->path_yaw = pyaw;
    new_node->cost += std::accumulate(course_lengths.begin(), course_lengths.end(), 0.0,
                                      [](double acc, double val)
                                      {
                                          return acc + std::abs(val);
                                      });

    new_node->parent = from_node;

    return new_node;
}

bool RRTStarDubins::check_collision(Node *node)
{
    if (node == NULL)
    {
        return false;
    }

    for (auto &obstacle : obstacle_list)
    {
        double ox = obstacle[0];
        double oy = obstacle[1];
        double size = obstacle[2];

        for (size_t i = 0; i < node->path_x.size(); ++i)
        {
            double dx = ox - node->path_x[i];
            double dy = oy - node->path_y[i];
            double d = dx * dx + dy * dy;

            if (d <= std::pow(size + robot_radius, 2))
            {
                return false; // collision
            }
        }
    }

    return true; // safe
}
std::vector<Node *> RRTStarDubins::find_near_nodes(Node *new_node)
{
    std::vector<Node *> nnodes;
    size_t nnode = node_list.size() + 1;
    double r = connect_circle_dist * std::sqrt(std::log(nnode) / nnode);

    for (Node *node : node_list)
    {
        double distance = std::pow(node->x - new_node->x, 2) + std::pow(node->y - new_node->y, 2);

        if (distance <= r * r)
        {
            nnodes.push_back(node);
        }
    }

    return nnodes;
}

double RRTStarDubins::calc_new_cost(Node *from_node, Node *to_node)
{
    auto [px, py, pyaw, mode, course_lengths] = dubbins_planner.plan_dubins_path(
        from_node->x, from_node->y, from_node->yaw,
        to_node->x, to_node->y, to_node->yaw, curvature);

    double cost = std::accumulate(course_lengths.begin(), course_lengths.end(), 0.0,
                                  [](double acc, double val)
                                  {
                                      return acc + std::abs(val);
                                  });

    return from_node->cost + cost;
}

Node *RRTStarDubins::choose_parent(Node *new_node, std::vector<Node *> nnodes)
{

    if (nnodes.size() == 0)
    {
        return NULL;
    }

    // Search nearest cost in near_inds
    std::vector<double> costs;
    for (Node *node : nnodes)
    {
        Node *t_node = steer(node, new_node);

        if (t_node && check_collision(t_node))
        {
            costs.push_back(calc_new_cost(node, new_node));
        }
        else
        {
            costs.push_back(std::numeric_limits<double>::infinity());
        }
    }

    auto min_cost_iter = std::min_element(costs.begin(), costs.end());
    double min_cost = *min_cost_iter;

    if (min_cost == std::numeric_limits<double>::infinity())
    {
        std::cout << "There is no good path. (min_cost is inf)\n";
        return NULL;
    }

    Node *min_node = nnodes[std::distance(costs.begin(), min_cost_iter)];
    new_node = steer(min_node, new_node);
    new_node->cost = min_cost;

    return new_node;
}

void RRTStarDubins::propagate_cost_to_leaves(Node *parent_node)
{
    for (Node *node : node_list)
    {
        if (node->parent == parent_node)
        {
            node->cost = calc_new_cost(parent_node, node);
            propagate_cost_to_leaves(node);
        }
    }
}

void RRTStarDubins::rewire(Node *new_node, std::vector<Node *> nnodes)
{
    for (Node *near_node : nnodes)
    {
        Node *edge_node = steer(new_node, near_node);

        if (!edge_node)
        {
            continue;
        }

        edge_node->cost = calc_new_cost(new_node, near_node);

        bool no_collision = check_collision(edge_node);
        bool improved_cost = near_node->cost > edge_node->cost;

        if (no_collision && improved_cost)
        {
            for (Node *node : node_list)
            {
                if (node->parent == near_node)
                {
                    node->parent = edge_node;
                }
            }
            near_node = edge_node;
            propagate_cost_to_leaves(near_node);
        }
    }
}

double RRTStarDubins::calc_dist_to_goal(double x, double y)
{
    double dx = x - end->x;
    double dy = y - end->y;
    return std::hypot(dx, dy);
}

Node *RRTStarDubins::search_best_goal_node()
{
    std::vector<Node *> goal_nodes;

    for (Node *node : node_list)
    {
        if (calc_dist_to_goal(node->x, node->y) <= goal_xy_th)
        {
            goal_nodes.push_back(node);
        }
    }

    // Angle check
    std::vector<Node *> final_goal_nodes;

    for (Node *node : goal_nodes)
    {
        if (std::abs(node->yaw - end->yaw) <= goal_yaw_th)
        {
            final_goal_nodes.push_back(node);
        }
    }

    if (final_goal_nodes.size() == 0)
    {
        return NULL;
    }

    double min_cost = std::numeric_limits<double>::infinity();
    Node *best_goal_node = NULL;

    for (Node *node : final_goal_nodes)
    {
        if (node->cost < min_cost)
        {
            min_cost = node->cost;
            best_goal_node = node;
        }
    }

    return best_goal_node;
}

std::vector<std::vector<double>> RRTStarDubins::generate_final_course(Node *goal_node)
{
    std::vector<std::vector<double>> path{{end->x, end->y}};
    Node *node = goal_node;

    while (node->parent)
    {
        for (size_t i = node->path_x.size(); i > 0; --i)
        {
            // std::cout << node->path_x[i - 1] << " , " << node->path_y[i - 1] << std::endl;
            path.push_back({node->path_x[i - 1], node->path_y[i - 1]});
        }
        node = node->parent;
    }

    path.push_back({start->x, start->y});
    return path;
}

std::vector<std::vector<double>> RRTStarDubins::planning(bool search_until_max_iter)
{
    node_list.push_back(start);

    for (int i = 0; i < max_iter; ++i)
    {
        // std::cout << "Iter: " << i << ", number of nodes: " << node_list.size() << std::endl;

        Node *rnd = get_random_node();
        Node *nearest_node = get_nearest_node(rnd);
        Node *new_node = steer(nearest_node, rnd);

        if (check_collision(new_node))
        {
            std::vector<Node *> near_nodes = find_near_nodes(new_node);
            new_node = choose_parent(new_node, near_nodes);

            if (new_node)
            {
                node_list.push_back(new_node);
                rewire(new_node, near_nodes);
            }
        }

        if (!search_until_max_iter && new_node)
        {
            Node *best_node = search_best_goal_node();
            if (best_node)
            {
                return generate_final_course(best_node);
            }
        }
    }

    // std::cout << "Reached max iteration" << std::endl;

    Node *best_node = search_best_goal_node();
    if (best_node)
    {
        return generate_final_course(best_node);
    }
    else
    {
        std::cout << "Cannot find path" << std::endl;
        return {};
    }
}
