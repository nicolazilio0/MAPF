#include "../../include/DijkstraSearch.hpp"

using namespace dijkstra;

// Check if two nodes are considered the same based on their coordinates
bool DijkstraSearch::is_same_node(double node_x, double node_y, Node *node_b)
{
    double dist = std::hypot(node_x - node_b->x, node_y - node_b->y);
    return dist <= 0.01; // Considered the same if distance is within a small threshold
}

// Check if two nodes are considered the same based on their coordinates
// TODO: Node pointer comparison does not works
bool DijkstraSearch::is_same_node(Node *node_a, Node *node_b)
{
    double dist = std::hypot(node_a->x - node_b->x, node_a->y - node_b->y);

    return dist <= 0.01;
}

// Find the ID of a target node in the node set based on its coordinates
int DijkstraSearch::find_id(Node *target_node)
{
    for (size_t i = 0; i < node_x.size(); i++)
    {
        if (is_same_node(node_x[i], node_y[i], target_node))
        {
            return i;
        }
    }

    return 0;
}

// Check if a node is present in a set
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

// Generate the final path from the close set and goal node
std::vector<std::vector<double>> DijkstraSearch::generate_final_path(tsl::ordered_map<int, Node *> &close_set, Node *goal_node)
{
    std::vector<std::vector<double>> path{{goal_node->x, goal_node->y}};
    int parent = goal_node->parent;

    // Reconstruct the path by backtracking from the goal node to the start
    while (parent != -1)
    {
        Node *node = close_set[parent];
        path.push_back({node->x, node->y});
        parent = node->parent;
    }

    return path;
}

// Perform Dijkstra's search algorithm to find the optimal path
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
        // Check if the goal is found in the close set
        if (has_node_in_set(close_set, goal))
        {
            std::cout << "GOAL is found!" << std::endl;
            goal->parent = current_node->parent;
            goal->cost = current_node->cost;
            break;
        }
        // Check if the open set is empty (no valid path found)
        else if (open_set.size() == 0)
        {
            std::cout << ":X cannot find valid path" << std::endl;
            break;
        }

        // Find the node with the minimum cost in the open set
        auto min_cost_it = std::min_element(
            open_set.begin(),
            open_set.end(),
            [](const auto &lhs, const auto &rhs)
            {
                return lhs.second->cost < rhs.second->cost;
            });

        int current_id = min_cost_it->first;

        current_node = open_set[current_id];

        open_set.erase(current_id);

        close_set[current_id] = current_node;

        // Explore neighbors and update costs in the open set
        for (size_t i = 0; i < edge_ids_list[current_id].size(); i++)
        {
            int n_id = edge_ids_list[current_id][i];
            double dx = node_x[n_id] - current_node->x;
            double dy = node_y[n_id] - current_node->y;

            double dist = std::hypot(dx, dy);
            Node *node = new Node(node_x[n_id], node_y[n_id], current_node->cost + dist, current_id);

            // Skip nodes already in the close set
            if (close_set.find(n_id) != close_set.end())
            {
                continue;
            }
            // Update node in open set if already present and with a lower cost
            if (open_set.find(n_id) != open_set.end())
            {
                if (open_set[n_id]->cost > node->cost)
                {
                    open_set[n_id] = node;
                }
            }
            else
            {
                open_set[n_id] = node; // Add node to open set if not present
            }
        }
    }

    // Generate and return the final path from the close set and goal node
    std::vector<std::vector<double>> path = generate_final_path(close_set, goal);

    return path;
}