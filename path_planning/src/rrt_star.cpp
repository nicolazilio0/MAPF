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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

#include "std_msgs/msg/header.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "../include/CoordinateMapper.hpp"
#include "DubinsPath.hpp"
#include "RRTStarDubins.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

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
    virtual std::vector<double> get_obstacle() const = 0;
};

struct PolygonObstacle : Obstacle
{
    geometry_msgs::msg::Polygon polygon;

    // approx the obstacle as a circle
    std::vector<double> get_obstacle() const override
    {

        // Initialize the bounding box coordinates
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();

        // Calculate the bounding box
        for (const auto &point : polygon.points)
        {
            min_x = std::min(min_x, static_cast<double>(point.x));
            max_x = std::max(max_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_y = std::max(max_y, static_cast<double>(point.y));
        }

        // Calculate the center of the bounding box
        double center_x = (min_x + max_x) / 2.0;
        double center_y = (min_y + max_y) / 2.0;

        // Calculate the radius (diameter/2) of the bounding circle
        double radius = std::sqrt((max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y)) / 2.0;

        // Return the vector containing center_x, center_y, and radius
        return {center_x, center_y, radius};
    }
};

struct CylinderObstacle : Obstacle
{
    double center_x;
    double center_y;
    double radius;

    std::vector<double> get_obstacle() const override
    {
        return {center_x, center_y, radius};
    }
};

struct MapBorder
{
    geometry_msgs::msg::Polygon polygon;

    // Threat the perimeter as a list of obstacles with a small radius
    std::vector<std::vector<double>> discretize_border(int discretization_point = 20, double radius = 0.01)
    {
        std::vector<std::vector<double>> points;

        for (size_t i = 0; i < polygon.points.size(); ++i)
        {
            const auto &start_point = polygon.points[i];
            const auto &end_point = polygon.points[(i + 1) % polygon.points.size()];

            // Calculate intermediate points along the edge for (int j = 0; j < numPointsPerEdge; ++j)
            for (int j = 0; j < discretization_point; ++j)
            {
                double x = start_point.x + (end_point.x - start_point.x) * static_cast<double>(j) / discretization_point;
                double y = start_point.y + (end_point.y - start_point.y) * static_cast<double>(j) / discretization_point;

                points.push_back({x, y, radius});
            }
        }

        return points;
    }
};

class RRTStarDubinsPlanner : public rclcpp::Node
{
public:
    RRTStarDubinsPlanner()
        : Node("rrtstardubins_planner"), coordinateMapper(20, 20, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        obstacles_subscritpion = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&RRTStarDubinsPlanner::get_obstacles, this, _1));
        gates_subscritpion = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", qos, std::bind(&RRTStarDubinsPlanner::get_gate, this, _1));
        map_subscritpion = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&RRTStarDubinsPlanner::get_map, this, _1));

        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0_odom = std::bind(&RRTStarDubinsPlanner::get_shelfino_position, this, _1, 0);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino1_odom = std::bind(&RRTStarDubinsPlanner::get_shelfino_position, this, _1, 1);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino2_odom = std::bind(&RRTStarDubinsPlanner::get_shelfino_position, this, _1, 2);

        shelfino0_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);
        shelfino1_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan1", 10);
        shelfino2_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan1", 10);

        shelfino0_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0_odom);
        shelfino1_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino1/odom", 10, shelfino1_odom);
        shelfino2_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino2/odom", 10, shelfino2_odom);

        obstacles_aquired = false;
        shelfinos_aquired = false;
        map_aquired = false;
        gate_aquired = false;

        rnd_min = -10;
        rnd_max = 10;

        planner_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RRTStarDubinsPlanner::generate_roadmap, this));
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscritpion;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino1_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino2_subscritpion;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino0_path_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino1_path_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino2_path_publisher;

    MapBorder mapBorder;
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    std::vector<Shelfino> shelfinos;
    Gate gate;

    CoordinateMapper coordinateMapper;
    DubinsPath dubinsPath;
    rclcpp::TimerBase::SharedPtr planner_timer;

    bool map_aquired;
    bool obstacles_aquired;
    bool gate_aquired;
    bool shelfinos_aquired;

    double rnd_min;
    double rnd_max;

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
        mapBorder.polygon = msg;

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
            RCLCPP_INFO(this->get_logger(), "Executing RRT*");

            // Stop the timer callback
            planner_timer->cancel();

            // Add obstacles to the obstacle list
            std::vector<std::vector<double>> circular_obstacles;

            for (const auto &obstacle_ptr : obstacles)
            {
                std::vector<double> obstacle = obstacle_ptr->get_obstacle();
                circular_obstacles.push_back(obstacle);
            }

            // Discretize map borders and add them to obstacle list
            std::vector<std::vector<double>> discretize_map_borders;

            for (std::vector<double> obstacle : mapBorder.discretize_border())
            {
                circular_obstacles.push_back(obstacle);
            }

            auto position = gate.position;
            auto orientation = gate.orientation;

            // Get shelfino's yaw angle
            double gate_t0 = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
            double gate_t1 = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
            double gate_yaw = std::atan2(gate_t0, gate_t1);
            // gate_yaw *= std::signbit(gate.position.y) ? -1 : 1;

            std::cout << "Gate orientation: " << gate_yaw << std::endl;

            rrtstar::Node *goal = new rrtstar::Node(position.x, position.y, gate_yaw);

            double total_duration = 0;

            // generate the path for each selfino

            for (const auto &shelfino : shelfinos)
            {
                auto id = shelfino.id;
                auto position = shelfino.position;
                auto orientation = shelfino.orientation;

                // Get shelfino's yaw angle
                double t0 = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
                double t1 = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
                double yaw = std::atan2(t0, t1);

                rrtstar::Node *start = new rrtstar::Node(position.x, position.y, yaw);

                // Create an instance of RRTStarDubins
                rrtstar::RRTStarDubins rrtStarDubins(start, goal, circular_obstacles, rnd_min, rnd_max);

                // Perform path planning
                std::vector<std::vector<double>> path;

                RCLCPP_INFO(this->get_logger(), "Shelfino %i", id);
                auto start_time = std::chrono::high_resolution_clock::now();

                // Repeat the planning until a non-empty path is obtained
                do
                {
                    path = rrtStarDubins.planning(false);
                } while (path.empty());

                auto stop_time = std::chrono::high_resolution_clock::now();

                auto duration = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count()) / 1000.0;
                total_duration += duration;
                RCLCPP_INFO(this->get_logger(), "Path for shelfino %i founded in: %f seconds", id, duration);

                // Iterate in reverse the path
                nav_msgs::msg::Path shelfino_path;
                shelfino_path.header.frame_id = "map";
                std::reverse(path.begin(), path.end());

                for (const auto &point : path)
                {

                    geometry_msgs::msg::PoseStamped pose_stmp;
                    // Get x,y
                    pose_stmp.pose.position.x = point[0];
                    pose_stmp.pose.position.y = point[1];

                    Eigen::Quaterniond quat;
                    quat = Eigen::AngleAxisd(point[2], Eigen::Vector3d::UnitZ());
                    // Get quaternion orientation from yaw
                    pose_stmp.pose.orientation.x = quat.x();
                    pose_stmp.pose.orientation.y = quat.y();
                    pose_stmp.pose.orientation.z = quat.z();
                    pose_stmp.pose.orientation.w = quat.w();

                    shelfino_path.poses.push_back(pose_stmp);
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
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rrtStarDubinsPlanner = std::make_shared<RRTStarDubinsPlanner>();
    rclcpp::spin(rrtStarDubinsPlanner);
    rclcpp::shutdown();
    return 0;
}