#include "../include/RRTStarDubins.hpp"

RRTStarDubins::RRTStarDubins(std::vector<double> start, std::vector<double> goal, std::vector<std::vector<double>> obstacle_list,
                             double min_rand, double max_rand, int goal_sample_rate, int max_iter,
                             double connect_circle_dist, double robot_radius)
    : start(start[0], start[1], start[2]),
      end(goal[0], goal[1], goal[2]),
      min_rand(min_rand),
      max_rand(max_rand),
      goal_sample_rate(goal_sample_rate),
      max_iter(max_iter),
      obstacle_list(obstacle_list),
      connect_circle_dist(connect_circle_dist),
      curvature(1.0),
      goal_yaw_th(0.0174532),
      goal_xy_th(0.5),
      robot_radius(robot_radius),
      dubbins_planner() // Assuming DubinsPath has a default constructor
{
}

Node RRTStarDubins::get_random_node()
{
    if (std::rand() % 100 > goal_sample_rate)
    {
        std::random_device rd;
        std::default_random_engine generator(rd());

        std::uniform_real_distribution<double> x_distribution(min_rand, max_rand);
        std::uniform_real_distribution<double> y_distribution(min_rand, max_rand);
        std::uniform_real_distribution<double> yaw_distribution(-M_PI, M_PI);

        double x = x_distribution(generator);
        double y = y_distribution(generator);
        double yaw = yaw_distribution(generator);

        Node rnd(x, y, yaw);
        return rnd;
    }
    else
    {
        return end;
    }
}

// Assuming node_list is a vector<Node>
size_t RRTStarDubins::get_nearest_node_index(const std::vector<Node> &node_list, const Node &rnd_node) const
{
    double min_dist = std::numeric_limits<double>::max();
    size_t min_ind = 0;

    for (size_t i = 0; i < node_list.size(); ++i)
    {
        double distance = std::pow(node_list[i].x - rnd_node.x, 2) + std::pow(node_list[i].y - rnd_node.y, 2);
        if (distance < min_dist)
        {
            min_dist = distance;
            min_ind = i;
        }
    }

    return min_ind;
}

Node *RRTStarDubins::steer(const Node &from_node, const Node &to_node)
{
    auto [px, py, pyaw, mode, course_lengths] = dubbins_planner.plan_dubins_path(
        from_node.x, from_node.y, from_node.yaw,
        to_node.x, to_node.y, to_node.yaw, curvature);

    if (px.size() <= 1)
    { // cannot find a Dubins path
        return nullptr;
    }

    Node *new_node = new Node(px.back(), py.back(), pyaw.back());
    new_node->path_x = px;
    new_node->path_y = py;
    new_node->path_yaw = pyaw;
    new_node->cost += std::accumulate(course_lengths.begin(), course_lengths.end(), 0.0);
    new_node->parent = &from_node;

    return new_node;
}

bool RRTStarDubins::check_collision(const Node *node, const std::vector<std::vector<double>> &obstacleList, double robot_radius) const
{
    if (node == nullptr)
    {
        return false;
    }

    for (const auto &obstacle : obstacleList)
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
std::vector<size_t> RRTStarDubins::find_near_nodes(const Node &new_node) const
{
    size_t nnode = node_list.size() + 1;
    double r = connect_circle_dist * std::sqrt(std::log(nnode) / nnode);

    std::vector<size_t> near_inds;

    for (size_t i = 0; i < node_list.size(); ++i)
    {
        double distance = std::pow(node_list[i].x - new_node.x, 2) + std::pow(node_list[i].y - new_node.y, 2);

        if (distance <= r * r)
        {
            near_inds.push_back(i);
        }
    }

    return near_inds;
}

double RRTStarDubins::calc_new_cost(const Node &from_node, const Node &to_node) const
{
    auto [px, py, pyaw, mode, course_lengths] = dubbins_planner.plan_dubins_path(
        from_node.x, from_node.y, from_node.yaw,
        to_node.x, to_node.y, to_node.yaw, curvature);

    double cost = std::accumulate(course_lengths.begin(), course_lengths.end(), 0.0,
                                  [](double acc, double val)
                                  {
                                      return acc + std::abs(val);
                                  });

    return from_node.cost + cost;
}

Node *RRTStarDubins::choose_parent(const Node &new_node, const std::vector<size_t> &near_inds)
{
    if (near_inds.empty())
    {
        return nullptr;
    }

    // Search nearest cost in near_inds
    std::vector<double> costs;
    for (size_t i : near_inds)
    {
        const Node &near_node = node_list[i];
        Node *t_node = steer(near_node, new_node);

        if (t_node && check_collision(t_node, obstacle_list, robot_radius))
        {
            costs.push_back(calc_new_cost(near_node, new_node));
        }
        else
        {
            costs.push_back(std::numeric_limits<double>::infinity()); // The cost of collision node
        }
    }

    auto min_cost_iter = std::min_element(costs.begin(), costs.end());
    double min_cost = *min_cost_iter;

    if (min_cost == std::numeric_limits<double>::infinity())
    {
        std::cout << "There is no good path. (min_cost is inf)\n";
        return nullptr;
    }

    size_t min_ind = near_inds[std::distance(costs.begin(), min_cost_iter)];
    Node *chosen_parent = steer(node_list[min_ind], new_node);
    chosen_parent->cost = min_cost;

    return chosen_parent;
}

void RRTStarDubins::propagate_cost_to_leaves(Node *parent_node)
{
    for (Node &node : node_list)
    {
        if (node.parent == parent_node)
        {
            node.cost = calc_new_cost(*parent_node, node);
            propagate_cost_to_leaves(&node);
        }
    }
}

void RRTStarDubins::rewire(const Node &new_node, const std::vector<size_t> &near_inds)
{
    for (size_t i : near_inds)
    {
        Node *near_node = &node_list[i];
        Node *edge_node = steer(new_node, *near_node);

        if (!edge_node)
        {
            continue;
        }

        edge_node->cost = calc_new_cost(new_node, *near_node);

        bool no_collision = check_collision(edge_node, obstacle_list, robot_radius);
        bool improved_cost = near_node->cost > edge_node->cost;

        if (no_collision && improved_cost)
        {
            for (Node &node : node_list)
            {
                if (node.parent == near_node)
                {
                    node.parent = edge_node;
                }
            }
            *near_node = *edge_node;
            propagate_cost_to_leaves(near_node);
        }
    }
}

double RRTStarDubins::calc_dist_to_goal(double x, double y) const
{
    double dx = x - end.x;
    double dy = y - end.y;
    return std::hypot(dx, dy);
}

int RRTStarDubins::search_best_goal_node() const
{
    std::vector<size_t> goal_indexes;

    for (size_t i = 0; i < node_list.size(); ++i)
    {
        const Node &node = node_list[i];
        if (calc_dist_to_goal(node.x, node.y) <= goal_xy_th)
        {
            goal_indexes.push_back(i);
        }
    }

    // Angle check
    std::vector<size_t> final_goal_indexes;

    for (size_t i : goal_indexes)
    {
        if (std::abs(node_list[i].yaw - end.yaw) <= goal_yaw_th)
        {
            final_goal_indexes.push_back(i);
        }
    }

    if (final_goal_indexes.empty())
    {
        return -1; // -1 is used to represent 'None'
    }

    double min_cost = std::numeric_limits<double>::infinity();
    int best_goal_index = -1;

    for (size_t i : final_goal_indexes)
    {
        if (node_list[i].cost < min_cost)
        {
            min_cost = node_list[i].cost;
            best_goal_index = static_cast<int>(i);
        }
    }

    return best_goal_index;
}

std::vector<std::vector<double>> RRTStarDubins::generate_final_course(int goal_index) const
{
    std::vector<std::vector<double>> path{{end.x, end.y}};
    const Node *node = &node_list[goal_index];

    while (node->parent != nullptr)
    {
        for (size_t i = node->path_x.size(); i > 0; --i)
        {
            std::cout << node->path_x[i - 1] << " , " << node->path_y[i - 1] << std::endl;
            path.push_back({node->path_x[i - 1], node->path_y[i - 1]});
        }
        node = node->parent;
    }

    path.push_back({start.x, start.y});
    return path;
}

std::vector<std::vector<double>> RRTStarDubins::planning(bool search_until_max_iter)
{
    node_list = {start};

    for (int i = 0; i < max_iter; ++i)
    {
        std::cout << "Iter: " << i << ", number of nodes: " << node_list.size() << std::endl;

        Node rnd = get_random_node();
        size_t nearest_ind = get_nearest_node_index(node_list, rnd);
        Node *new_node = steer(node_list[nearest_ind], rnd);

        if (check_collision(new_node, obstacle_list, robot_radius))
        {
            std::vector<size_t> near_indexes = find_near_nodes(*new_node);
            new_node = choose_parent(*new_node, near_indexes);

            if (new_node)
            {
                node_list.push_back(*new_node);
                rewire(*new_node, near_indexes);
            }
        }

        if (!search_until_max_iter && new_node)
        { // Check reaching the goal
            int last_index = search_best_goal_node();
            if (last_index != -1)
            {
                return generate_final_course(last_index);
            }
        }
    }

    std::cout << "Reached max iteration" << std::endl;

    int last_index = search_best_goal_node();
    if (last_index != -1)
    {
        return generate_final_course(last_index);
    }
    else
    {
        std::cout << "Cannot find path" << std::endl;
        return {};
    }
}
