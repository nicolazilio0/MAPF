#include "../include/VoronoiDijkstra.hpp"

VoronoiDijkstra::VoronoiDijkstra(std::vector<float> voronoi_x, std::vector<float> voronoi_y, float robot_radius_) : robot_radius(robot_radius_),
                                                                                                                    sample_x(voronoi_x),
                                                                                                                    sample_y(voronoi_y){};

bool VoronoiDijkstra::check_collision(float s_x, float s_y, float g_x, float g_y, Kdtree::KdTree *obstalces_tree)
{
    float x = s_x;
    float y = s_y;

    float dx = g_x - s_x;
    float dy = g_y - s_y;
    float yaw = std::atan2(dy, dx);
    float d = std::hypot(dx, dy);

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

        float dist = nn.dist;

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

    float dist = nn.dist;

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
        float i_x = sample_x[i];
        float i_y = sample_y[i];

        Kdtree::KdNodeVector knn;
        std::vector<double> query = {i_x, i_y};

        nodes_tree.k_nearest_neighbors(query, n_sample, &knn);

        std::vector<int> edge_id;

        for (int j = 1; j < n_sample; j++)
        {
            auto const &nn = knn[j];
            float n_x = sample_x[nn.index];
            float n_y = sample_y[nn.index];

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

std::vector<std::vector<float>> VoronoiDijkstra::planning(float s_x, float s_y, float g_x, float g_y, std::vector<float> o_x, std::vector<float> o_y)
{
    std::vector<std::vector<float>> path;

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

    return path;
}