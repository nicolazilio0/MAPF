#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <cmath>
#include <fstream>
#include <chrono>

#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

#include "std_msgs/msg/header.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "../include/CoordinateMapper.hpp"
#include "../include/VoronoiDijkstra.hpp"

#include <boost/polygon/voronoi.hpp>

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
static const rmw_qos_profile_t rmw_qos_profile_custom =
    {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};

struct Shelfino
{
    int id;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
};

struct Gate
{
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
};

struct Obstacle
{
    virtual geometry_msgs::msg::Polygon get_polygon() const = 0;
};

struct PolygonObstacle : Obstacle
{
    geometry_msgs::msg::Polygon polygon;

    // Discretize the polygon to have more viapoints in Voronoi diagram and avoid collision
    geometry_msgs::msg::Polygon get_polygon() const override
    {
        geometry_msgs::msg::Polygon discr_polygon;
        // std::cout << "Polygono" << std::endl;

        for (size_t i = 0; i < polygon.points.size(); i++)
        {
            const auto &start_point = polygon.points[i];
            const auto &end_point = polygon.points[(i + 1) % polygon.points.size()];

            double dx = end_point.x - start_point.x;
            double dy = end_point.y - start_point.y;
            double dist = std::hypot(dx, dy);

            int discretization_point = (dist / 0.4) + 1;

            // Calculate intermediate points along the edge for (int j = 0; j < numPointsPerEdge; ++j)
            for (int j = 0; j < discretization_point; j++)
            {
                double x = start_point.x + (end_point.x - start_point.x) * static_cast<double>(j) / discretization_point;
                double y = start_point.y + (end_point.y - start_point.y) * static_cast<double>(j) / discretization_point;

                geometry_msgs::msg::Point32 point;
                point.x = static_cast<float>(x);
                point.y = static_cast<float>(y);

                discr_polygon.points.push_back(point);
            }
        }

        return discr_polygon;
    }
};

struct CylinderObstacle : Obstacle
{
    double center_x;
    double center_y;
    double radius;

    // discretize the circumference as a set of segments to generate the voronoi diagram
    // this is also needed to avoid osbtacles since the planner works with obstalces discretized in points
    geometry_msgs::msg::Polygon get_polygon() const override
    {
        int num_segments = (2 * M_PI * radius) / (0.3);

        geometry_msgs::msg::Polygon polygon;

        for (int i = 0; i < num_segments; i++)
        {
            double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_segments);
            double x = center_x + radius * std::cos(theta);
            double y = center_y + radius * std::sin(theta);

            geometry_msgs::msg::Point32 point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);

            polygon.points.push_back(point);
        }

        // Add the first point to complete the circle
        polygon.points.push_back(polygon.points.front());

        return polygon;
    }
};

// User define point and segment to generate voronoi diagram

struct Point
{
    int a;
    int b;
    Point(int x, int y) : a(x), b(y) {}
};

struct Segment
{
    Point p0;
    Point p1;
    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost
{
    namespace polygon
    {

        template <>
        struct geometry_concept<Point>
        {
            typedef point_concept type;
        };

        template <>
        struct point_traits<Point>
        {
            typedef int coordinate_type;

            static inline coordinate_type get(
                const Point &point, orientation_2d orient)
            {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };

        template <>
        struct geometry_concept<Segment>
        {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<Segment>
        {
            typedef int coordinate_type;
            typedef Point point_type;

            static inline point_type get(const Segment &segment, direction_1d dir)
            {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    } // polygon
} // boost

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class VoronoiPlanner : public rclcpp::Node
{
public:
    VoronoiPlanner()
        : Node("vornoi_planner"), coordinateMapper(20, 20, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        obstacles_subscritpion = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&VoronoiPlanner::get_obstacles, this, _1));
        gates_subscritpion = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", qos, std::bind(&VoronoiPlanner::get_gate, this, _1));
        map_subscritpion = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&VoronoiPlanner::get_map, this, _1));

        voronoi_publisher = this->create_publisher<geometry_msgs::msg::Polygon>("voronoi", 10);

        shelfino0_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);
        shelfino1_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan1", 10);
        shelfino2_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan1", 10);

        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0_odom = std::bind(&VoronoiPlanner::get_shelfino_position, this, _1, 0);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino1_odom = std::bind(&VoronoiPlanner::get_shelfino_position, this, _1, 1);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino2_odom = std::bind(&VoronoiPlanner::get_shelfino_position, this, _1, 2);
        shelfino0_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0_odom);
        shelfino1_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino1/odom", 10, shelfino1_odom);
        shelfino2_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino2/odom", 10, shelfino2_odom);

        obstacles_aquired = false;
        shelfinos_aquired = false;
        map_aquired = false;
        gate_aquired = false;

        planner_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VoronoiPlanner::generate_roadmap, this));
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscritpion;

    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr voronoi_publisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino1_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino2_subscritpion;

    PolygonObstacle mapBorders;
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    std::vector<Shelfino> shelfinos;
    Gate gate;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino0_path_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino1_path_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino2_path_publisher;

    CoordinateMapper coordinateMapper;
    rclcpp::TimerBase::SharedPtr planner_timer;

    bool map_aquired;
    bool obstacles_aquired;
    bool gate_aquired;
    bool shelfinos_aquired;

    // Callbacks for topic handling
    void get_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
    {

        RCLCPP_INFO(this->get_logger(), "Received %zu obstacles", msg.obstacles.size());

        for (size_t i = 0; i < msg.obstacles.size(); i++)
        {
            const auto &msg_obstacle = msg.obstacles[i];

            // Access the polygon and radius fields for each obstacle
            const auto &polygon = msg_obstacle.polygon;
            const auto &radius = msg_obstacle.radius;

            if (radius == 0.0)
            {
                auto obstacle = std::make_unique<PolygonObstacle>();
                obstacle->polygon = polygon;
                obstacles.push_back(std::move(obstacle));
            }
            else
            {
                // Circle only contains one polygon
                const auto &point = polygon.points[0];
                auto obstacle = std::make_unique<CylinderObstacle>();
                obstacle->center_x = point.x;
                obstacle->center_y = point.y;
                obstacle->radius = radius;
                obstacles.push_back(std::move(obstacle));
            }
        }

        obstacles_aquired = true;
    }

    void get_gate(const geometry_msgs::msg::PoseArray &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received gate");

        const auto &pose = msg.poses[0];
        gate.position = pose.position;
        gate.orientation = pose.orientation;

        gate_aquired = true;
    }

    void get_map(const geometry_msgs::msg::Polygon &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map borders");

        mapBorders.polygon = msg;

        map_aquired = true;
    }

    void get_shelfino_position(const nav_msgs::msg::Odometry::SharedPtr &msg, int robotId)
    {
        auto it = std::find_if(shelfinos.begin(), shelfinos.end(),
                               [robotId](const Shelfino &shelfino)
                               { return shelfino.id == robotId; });

        if (it == shelfinos.end())
        {
            RCLCPP_INFO(this->get_logger(), "Received Shelfino %d odometry", robotId);

            Shelfino shelfino;
            shelfino.id = robotId;
            shelfinos.push_back(shelfino);
            it = std::prev(shelfinos.end());
        }

        it->position = msg->pose.pose.position;
        it->orientation = msg->pose.pose.orientation;

        // Check if messages have been received for all three robots
        if (shelfinos.size() == 3)
        {
            // Unsubscribe from the topics
            shelfino0_subscritpion.reset();
            shelfino1_subscritpion.reset();
            shelfino2_subscritpion.reset();

            RCLCPP_INFO(this->get_logger(), "Recived all Shelfinos");

            shelfinos_aquired = true;
        }
    }

    void generate_roadmap()
    {
        if (obstacles_aquired && gate_aquired && shelfinos_aquired && map_aquired)
        {
            RCLCPP_INFO(this->get_logger(), "Executing Voronoi");
            planner_timer->cancel();

            std::vector<double> o_x;
            std::vector<double> o_y;

            // Create a list of segments to discretize polygons and circles
            std::vector<Segment> segments;
            int image1_x, image1_y, image2_x, image2_y;

            // Convert obstacles and map borders
            for (const auto &obstacle : obstacles)
            {
                const auto &ros_polygon = obstacle->get_polygon();

                for (size_t i = 0; i < ros_polygon.points.size() - 1; ++i)
                {
                    const auto &point1 = ros_polygon.points[i];
                    const auto &point2 = ros_polygon.points[i + 1];

                    o_x.push_back(point1.x);
                    o_x.push_back(point2.x);
                    o_y.push_back(point1.y);
                    o_y.push_back(point2.y);

                    coordinateMapper.gazebo2img(point1.x, point1.y, image1_x, image1_y);
                    coordinateMapper.gazebo2img(point2.x, point2.y, image2_x, image2_y);

                    segments.push_back(Segment(image1_x, image1_y, image2_x, image2_y));
                }

                // Connect the last point with the first one
                const auto &first_point = ros_polygon.points.front();
                const auto &last_point = ros_polygon.points.back();
                coordinateMapper.gazebo2img(first_point.x, first_point.y, image1_x, image1_y);
                coordinateMapper.gazebo2img(last_point.x, last_point.y, image2_x, image2_y);

                segments.push_back(Segment(image1_x, image1_y, image2_x, image2_y));
            }

            // Create segments for map borders

            auto map_borders = mapBorders.get_polygon();

            for (size_t i = 0; i < map_borders.points.size() - 1; ++i)
            {
                const auto &point1 = map_borders.points[i];
                const auto &point2 = map_borders.points[i + 1];

                o_x.push_back(point1.x);
                o_x.push_back(point2.x);
                o_y.push_back(point1.y);
                o_y.push_back(point2.y);

                coordinateMapper.gazebo2img(point1.x, point1.y, image1_x, image1_y);
                coordinateMapper.gazebo2img(point2.x, point2.y, image2_x, image2_y);

                segments.push_back(Segment(image1_x, image1_y, image2_x, image2_y));
            }

            // Connect the last point of map borders with the first one
            const auto &first_map_point = map_borders.points.front();
            const auto &last_map_point = map_borders.points.back();
            coordinateMapper.gazebo2img(first_map_point.x, first_map_point.y, image1_x, image1_y);
            coordinateMapper.gazebo2img(last_map_point.x, last_map_point.y, image2_x, image2_y);

            segments.push_back(Segment(image1_x, image1_y, image2_x, image2_y));

            // Create the Voronoi diagram
            boost::polygon::voronoi_diagram<double> vd;
            boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

            // Send the diagram to the gui

            geometry_msgs::msg::Polygon voronoi_polygon;

            // Generate the path

            std::vector<double> voronoi_x;
            std::vector<double> voronoi_y;

            double gazebo_x;
            double gazebo_y;

            // get only the voronoi's vertices
            for (auto it = vd.vertices().begin(); it != vd.vertices().end(); ++it)
            {
                const auto &vertex = *it;

                coordinateMapper.img2gazebo(vertex.x(), vertex.y(), gazebo_x, gazebo_y);
                // filter vertex out of workspace
                if (gazebo_x <= 10 && gazebo_x >= -10 && gazebo_y <= 10 && gazebo_y >= -10)
                {
                    voronoi_x.push_back(gazebo_x);
                    voronoi_y.push_back(gazebo_y);

                    geometry_msgs::msg::Point32 point;
                    point.x = gazebo_x;
                    point.y = gazebo_y;
                    voronoi_polygon.points.push_back(point);
                }
            }

            // publish voronoi diagram to map visualizer
            voronoi_publisher->publish(voronoi_polygon);

            RCLCPP_INFO(this->get_logger(), "Executing Voronoi + Dijkstra");

            auto gate_pos = gate.position;
            double total_duration = 0;

            // generate the path for each selfino
            for (const auto &shelfino : shelfinos)
            {
                auto id = shelfino.id;
                auto shelfino_pos = shelfino.position;

                RCLCPP_INFO(this->get_logger(), "Shelfino %i", id);
                auto start_time = std::chrono::high_resolution_clock::now();

                VoronoiDijkstra voronoiDijkstra(voronoi_x, voronoi_y);
                std::vector<std::vector<double>> path = voronoiDijkstra.planning(shelfino_pos.x, shelfino_pos.y, gate_pos.x, gate_pos.y, o_x, o_y);
                if (path.size() == 0)
                {
                    // interrupt the shelfino planning process if path is not founded
                    break;
                }
                auto stop_time = std::chrono::high_resolution_clock::now();

                auto duration = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count()) / 1000.0;
                total_duration += duration;
                RCLCPP_INFO(this->get_logger(), "Path for shelfino %i founded in: %f seconds", id, duration);

                // Iterate in reverse the path
                nav_msgs::msg::Path shelfino_path;
                shelfino_path.header.frame_id = "map";
                std::reverse(path.begin(), path.end());

                for (size_t i = 0; i < path.size() - 1; ++i)
                {
                    const auto &start_point = path[i];
                    const auto &end_point = path[i + 1];

                    double dx = end_point[0] - start_point[0];
                    double dy = end_point[1] - start_point[1];
                    double dist = std::hypot(dx, dy);
                    // try to get the robot orientatio through viapoints
                    double yaw = std::atan2(dy, dx);

                    int discretization_point = (dist / 0.1); // discretize the path in 0.1m steps
                    // this is need since the collision avoidance alg. in the orchestrator works with this step size

                    for (int j = 0; j < discretization_point; j++)
                    {
                        double x = start_point[0] + (end_point[0] - start_point[0]) * static_cast<double>(j) / discretization_point;
                        double y = start_point[1] + (end_point[1] - start_point[1]) * static_cast<double>(j) / discretization_point;

                        geometry_msgs::msg::PoseStamped pose_stmp;

                        pose_stmp.pose.position.x = x;
                        pose_stmp.pose.position.y = y;

                        Eigen::Quaterniond quat;
                        quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

                        pose_stmp.pose.orientation.x = quat.x();
                        pose_stmp.pose.orientation.y = quat.y();
                        pose_stmp.pose.orientation.z = quat.z();
                        pose_stmp.pose.orientation.w = quat.w();

                        shelfino_path.poses.push_back(pose_stmp);
                        shelfino_path.poses.push_back(pose_stmp);
                    }
                }

                // Sent the path accordingly to the shelfino's id
                switch (id)
                {
                case 0:
                    shelfino0_path_publisher->publish(shelfino_path);
                    break;
                case 1:
                    shelfino1_path_publisher->publish(shelfino_path);
                    break;
                case 2:
                    shelfino2_path_publisher->publish(shelfino_path);
                    break;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Total roadmap planning time: %f seconds", total_duration);
        }
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto voronoiPlanner = std::make_shared<VoronoiPlanner>();
    rclcpp::spin(voronoiPlanner);
    rclcpp::shutdown();
    return 0;
}