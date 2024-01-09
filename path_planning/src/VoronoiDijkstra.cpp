#include "../include/VoronoiDijkstra.hpp"

VoronoiDijkstra::VoronoiDijkstra(std::vector<double> voronoi_x, std::vector<double> voronoi_y, double robot_radius_) : robot_radius(robot_radius_),
                                                                                                                       sample_x(voronoi_x),
                                                                                                                       sample_y(voronoi_y){};

bool VoronoiDijkstra::check_collision(double s_x, double s_y, double g_x, double g_y, Kdtree::KdTree *obstalces_tree)
{
    double x = s_x;
    double y = s_y;

    double dx = g_x - s_x;
    double dy = g_y - s_y;
    double yaw = std::atan2(dy, dx);
    double d = std::hypot(dx, dy);

    if (d >= MAX_EDGE_LEN)
    {
        return true;
    }

    int n_step = std::round(d / robot_radius);

    for (int i = 0; i < n_step; i++)
    {
        Kdtree::KdNodeVector knn;
        std::vector<double> query = {x, y};

        obstalces_tree->k_nearest_neighbors(query, 1, &knn);

        const auto &nn = knn[0];

        double dist = nn.dist;

        if (dist <= robot_radius)
        {
            return true;
        }

        x += robot_radius * std::cos(yaw);
        y += robot_radius * std::sin(yaw);
    }

    Kdtree::KdNodeVector knn;
    std::vector<double> query = {g_x, g_y};

    obstalces_tree->k_nearest_neighbors(query, 1, &knn);

    const auto &nn = knn[0];

    double dist = nn.dist;

    if (dist <= robot_radius)
    {
        return true;
    }

    return false;
}

std::vector<std::vector<int>> VoronoiDijkstra::generate_roadmap_info(Kdtree::KdTree *obstalces_tree)
{
    std::vector<std::vector<int>> roadmap;
    int n_sample = sample_x.size();

    Kdtree::KdNodeVector nodes;

    for (int i = 0; i < n_sample; i++)
    {
        nodes.push_back(Kdtree::KdNode({sample_x[i], sample_y[i]}));
    }

    Kdtree::KdTree nodes_tree(&nodes);

    for (int i = 0; i < n_sample; i++)
    {
        double i_x = sample_x[i];
        double i_y = sample_y[i];

        Kdtree::KdNodeVector knn;
        std::vector<double> query = {i_x, i_y};

        nodes_tree.k_nearest_neighbors(query, n_sample, &knn);

        std::vector<int> edge_id;

        for (size_t j = 1; j < knn.size(); j++)
        {
            auto const &nn = knn[j];
            double n_x = sample_x[nn.index];
            double n_y = sample_y[nn.index];

            if (!check_collision(i_x, i_y, n_x, n_y, obstalces_tree))
            {
                edge_id.push_back(nn.index);
            }

            if (edge_id.size() >= N_KNN)
            {
                break;
            }
        }

        roadmap.push_back(edge_id);
    }

    return roadmap;
}

std::vector<std::vector<double>> VoronoiDijkstra::planning(double s_x, double s_y, double g_x, double g_y, std::vector<double> o_x, std::vector<double> o_y)
{

    Kdtree::KdNodeVector obstalces;

    for (std::size_t i = 0; i < o_x.size(); i++)
    {
        obstalces.push_back(Kdtree::KdNode({o_x[i], o_y[i]}));
    }

    Kdtree::KdTree obstalces_tree(&obstalces);

    sample_x.push_back(s_x);
    sample_y.push_back(s_y);
    sample_x.push_back(g_x);
    sample_y.push_back(g_y);

    std::vector<std::vector<int>> road_map_info = generate_roadmap_info(&obstalces_tree);

    dijkstra::DijkstraSearch dijkstra(s_x, s_y, g_x, g_y, sample_x, sample_y, road_map_info);

    std::vector<std::vector<double>> path = dijkstra.search();

    return path;
}