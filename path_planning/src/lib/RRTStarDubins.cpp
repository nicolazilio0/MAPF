#include "../../include/RRTStarDubins.hpp"

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
      curvature(2),
      goal_yaw_th(0.0174532),
      goal_xy_th(0.05),
      robot_radius(robot_radius),
      dubbins_planner()
{
}

// Generates a random node in the workspace, considering the goal with a certain range
Node *RRTStarDubins::get_random_node()
{
    std::uniform_int_distribution<int> int_distribution(0, 100);
    if (int_distribution(generator) > goal_sample_rate)
    {
        // Generate random x, y, and yaw values
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
        // Return the goal node with a certain probability
        return end;
    }
}

// Finds the nearest node in the tree to a given random node
Node *RRTStarDubins::get_nearest_node(Node *rnd_node)
{
    double min_dist = std::numeric_limits<double>::max();
    Node *nearest_node;

    // Iterate through all nodes in the tree
    for (Node *node : node_list)
    {
        // Calculate Euclidean distance
        double distance = std::pow(node->x - rnd_node->x, 2) + std::pow(node->y - rnd_node->y, 2);
        if (distance < min_dist)
        {
            min_dist = distance;
            nearest_node = node;
        }
    }

    return nearest_node;
}

// Steers from one node to another and returns the new node if a valid Dubins path is found
Node *RRTStarDubins::steer(Node *from_node, Node *to_node)
{
    // Plan a Dubins path from 'from_node' to 'to_node'
    auto [px, py, pyaw, mode, course_lengths] = dubbins_planner.plan_dubins_path(
        from_node->x, from_node->y, from_node->yaw,
        to_node->x, to_node->y, to_node->yaw, curvature);

    // If the Dubins path is not found, return nullptr
    if (px.size() <= 1)
    { // cannot find a Dubins path
        return nullptr;
    }

    // Create a new node at the end of the Dubins path
    Node *new_node = new Node(px.back(), py.back(), pyaw.back());
    new_node->path_x = px;
    new_node->path_y = py;
    new_node->path_yaw = pyaw;

    // Calculate the cost of the new node based on the Dubins path lengths
    new_node->cost += std::accumulate(course_lengths.begin(), course_lengths.end(), 0.0,
                                      [](double acc, double val)
                                      {
                                          return acc + std::abs(val);
                                      });

    new_node->parent = from_node;

    return new_node;
}

// Checks collision for a given node's path with obstacles
bool RRTStarDubins::check_collision(Node *node)
{
    if (node == NULL)
    {
        return false;
    }

    // Iterate through obstacles and check collision for each point in the node's path
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

            // Check collision
            if (d <= std::pow(size + robot_radius, 2))
            {
                return false; // collision
            }
        }
    }

    return true; // safe
}

// Finds nodes near the given new node within a certain radius
std::vector<Node *> RRTStarDubins::find_near_nodes(Node *new_node)
{
    std::vector<Node *> nnodes;
    size_t nnode = node_list.size() + 1;
    double r = connect_circle_dist * std::sqrt(std::log(nnode) / nnode);

    // Iterate through all nodes and find those within the radius
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

// Calculates the new cost of reaching 'to_node' from 'from_node' using Dubins path lengths
double RRTStarDubins::calc_new_cost(Node *from_node, Node *to_node)
{
    auto [px, py, pyaw, mode, course_lengths] = dubbins_planner.plan_dubins_path(
        from_node->x, from_node->y, from_node->yaw,
        to_node->x, to_node->y, to_node->yaw, curvature);

    // Calculate the cost based on Dubins path lengths
    double cost = std::accumulate(course_lengths.begin(), course_lengths.end(), 0.0,
                                  [](double acc, double val)
                                  {
                                      return acc + std::abs(val);
                                  });

    return from_node->cost + cost;
}

// Chooses the parent node for the new node among the nearby nodes with the lowest cost
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

        // If a valid Dubins path is found and there is no collision, calculate the cost
        if (t_node && check_collision(t_node))
        {
            costs.push_back(calc_new_cost(node, new_node));
        }
        else
        {
            costs.push_back(std::numeric_limits<double>::infinity());
        }
    }

    // Find the node with the minimum cost
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

// Propagates the cost to leaves starting from the parent node
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

// Rewires the tree considering the new node and nearby nodes
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

        // Check collision and compare costs
        bool no_collision = check_collision(edge_node);
        bool improved_cost = near_node->cost > edge_node->cost;

        // If there is no collision and the cost is improved, rewire the tree
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

// Calculates the Euclidean distance from a point to the goal
double RRTStarDubins::calc_dist_to_goal(double x, double y)
{
    double dx = x - end->x;
    double dy = y - end->y;
    return std::hypot(dx, dy);
}

// Searches for the best goal node among nodes within the goal region
Node *RRTStarDubins::search_best_goal_node()
{
    std::vector<Node *> goal_nodes;

    // Find nodes within the goal region based on xy coordinates
    for (Node *node : node_list)
    {
        if (calc_dist_to_goal(node->x, node->y) <= goal_xy_th)
        {
            goal_nodes.push_back(node);
        }
    }

    // Check angle compatibility
    std::vector<Node *> final_goal_nodes;

    for (Node *node : goal_nodes)
    {
        if (std::abs(node->yaw - end->yaw) <= goal_yaw_th)
        {
            final_goal_nodes.push_back(node);
        }
    }

    // If no compatible goal nodes are found, return NULL
    if (final_goal_nodes.size() == 0)
    {
        return NULL;
    }

    // Find the goal node with the minimum cost
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

// Generates the final path from the start to the goal node
std::vector<std::vector<double>> RRTStarDubins::generate_final_course(Node *goal_node)
{
    std::vector<std::vector<double>> path{{end->x, end->y, end->yaw}};
    Node *node = goal_node;

    // Reconstruct the path by backtracking from the goal node to the start
    while (node->parent)
    {
        for (size_t i = node->path_x.size(); i > 0; --i)
        {
            path.push_back({node->path_x[i - 1], node->path_y[i - 1], node->path_yaw[i - 1]});
        }
        node = node->parent;
    }

    path.push_back({start->x, start->y, start->yaw});
    return path;
}

// The main planning function that runs the RRT* algorithm
std::vector<std::vector<double>> RRTStarDubins::planning(bool search_until_max_iter)
{
    // Initialize the node list with the start node
    node_list.push_back(start);

    // Iterate for a maximum number of iterations
    for (int i = 0; i < max_iter; ++i)
    {
        Node *rnd = get_random_node();              // Generate a random node
        Node *nearest_node = get_nearest_node(rnd); // Find the nearest node in the tree
        Node *new_node = steer(nearest_node, rnd);  // Steer towards the random node

        // Check collision for the new node's path
        if (check_collision(new_node))
        {
            // Find nearby nodes and choose a parent for the new node
            std::vector<Node *> near_nodes = find_near_nodes(new_node); // Add the new node to the tree
            new_node = choose_parent(new_node, near_nodes);             // Rewire the tree considering the new node

            if (new_node)
            {
                node_list.push_back(new_node);
                rewire(new_node, near_nodes);
            }
        }

        // If searching until the maximum iteration is not required and a new node is found,
        // check if the goal is reached and return the final path
        if (!search_until_max_iter && new_node)
        {
            Node *best_node = search_best_goal_node();
            if (best_node)
            {
                return generate_final_course(best_node);
            }
        }
    }

    // If maximum iterations are reached, or no valid path is found, return an empty path
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
