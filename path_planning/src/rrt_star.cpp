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

#include "CoordinateMapper.hpp"
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
    virtual std::vector<double> getObstacle() const = 0;
};

struct PolygonObstacle : Obstacle
{
    geometry_msgs::msg::Polygon polygon;

    std::vector<double> getObstacle() const override
    {

        // Initialize the bounding box coordinates
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        // Calculate the bounding box
        for (const auto &point : polygon.points)
        {
            minX = std::min(minX, static_cast<double>(point.x));
            maxX = std::max(maxX, static_cast<double>(point.x));
            minY = std::min(minY, static_cast<double>(point.y));
            maxY = std::max(maxY, static_cast<double>(point.y));
        }

        // Calculate the center of the bounding box
        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        // Calculate the radius (diameter/2) of the bounding circle
        double radius = std::sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY)) / 2.0;

        // Return the vector containing center_x, center_y, and radius
        return {centerX, centerY, radius};
    }
};

struct CylinderObstacle : Obstacle
{
    double centerX;
    double centerY;
    double radius;

    std::vector<double> getObstacle() const override
    {
        return {centerX, centerY, radius};
    }
};

struct MapBorder
{
    geometry_msgs::msg::Polygon polygon;

    std::vector<std::vector<double>> discretizeBorder(int discretizationPoint = 20, double radius = 0.01)
    {
        std::vector<std::vector<double>> points;

        for (size_t i = 0; i < polygon.points.size(); ++i)
        {
            const auto &startPoint = polygon.points[i];
            const auto &endPoint = polygon.points[(i + 1) % polygon.points.size()];

            // Calculate intermediate points along the edge for (int j = 0; j < numPointsPerEdge; ++j)
            for (int j = 0; j < discretizationPoint; ++j)
            {
                double x = startPoint.x + (endPoint.x - startPoint.x) * static_cast<double>(j) / discretizationPoint;
                double y = startPoint.y + (endPoint.y - startPoint.y) * static_cast<double>(j) / discretizationPoint;

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
        : Node("rrtstardubins_planner"), coordinateMapper(17, 17, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&RRTStarDubinsPlanner::getObstacles, this, _1));
        gatesSubscritpion_ = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", qos, std::bind(&RRTStarDubinsPlanner::getGates, this, _1));
        mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&RRTStarDubinsPlanner::getMap, this, _1));

        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0Odom = std::bind(&RRTStarDubinsPlanner::getShelfinoPosition, this, _1, 0);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino1Odom = std::bind(&RRTStarDubinsPlanner::getShelfinoPosition, this, _1, 1);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino2Odom = std::bind(&RRTStarDubinsPlanner::getShelfinoPosition, this, _1, 2);

        shelfino0PathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("shelfino0/rrtstar_path", 10);

        shelfino0Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0Odom);
        shelfino1Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino1/odom", 10, shelfino1Odom);
        shelfino2Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino2/odom", 10, shelfino2Odom);

        obstaclesAquired = false;
        shelfinosAquired = false;
        mapAquired = false;
        gateAquired = false;

        rndMin = -9;
        rndMax = 9;

        plannerTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RRTStarDubinsPlanner::generateRoadmap, this));
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscritpion_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0Subscritpion_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino1Subscritpion_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino2Subscritpion_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino0PathPublisher_;

    MapBorder mapBorder;
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    std::vector<Shelfino> shelfinos;
    Gate gate;

    CoordinateMapper coordinateMapper;
    DubinsPath dubinsPath;
    rclcpp::TimerBase::SharedPtr plannerTimer_;

    bool mapAquired;
    bool obstaclesAquired;
    bool gateAquired;
    bool shelfinosAquired;

    double rndMin;
    double rndMax;

    // Callbacks for topic handling
    void getObstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
    {

        RCLCPP_INFO(this->get_logger(), "Received %zu obstacles", msg.obstacles.size());

        for (size_t i = 0; i < msg.obstacles.size(); i++)
        {
            const auto &msgObstacle = msg.obstacles[i];

            // Access the polygon and radius fields for each obstacle
            const auto &polygon = msgObstacle.polygon;
            const auto &radius = msgObstacle.radius;

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
                obstacle->centerX = point.x;
                obstacle->centerY = point.y;
                obstacle->radius = radius / 2.0;
                obstacles.push_back(std::move(obstacle));
            }
        }

        obstaclesAquired = true;
    }
    void getGates(const geometry_msgs::msg::PoseArray &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received gate");

        const auto &pose = msg.poses[0];
        gate.position = pose.position;
        gate.orientation = pose.orientation;

        gateAquired = true;
    }

    void getMap(const geometry_msgs::msg::Polygon &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map borders");
        mapBorder.polygon = msg;

        mapAquired = true;
    }

    void getShelfinoPosition(const nav_msgs::msg::Odometry::SharedPtr &msg, int robotId)
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
            shelfino0Subscritpion_.reset();
            shelfino1Subscritpion_.reset();
            shelfino2Subscritpion_.reset();

            RCLCPP_INFO(this->get_logger(), "Recived all Shelfinos");

            shelfinosAquired = true;
        }
    }

    void generateRoadmap()
    {
        if (obstaclesAquired && gateAquired && shelfinosAquired && mapAquired)
        {
            RCLCPP_INFO(this->get_logger(), "Executing RRT*");

            // Stop the timer callback
            plannerTimer_->cancel();

            // Add obstacles to the obstacle list
            std::vector<std::vector<double>> circular_obstacles;

            for (const auto &obstaclePtr : obstacles)
            {
                std::vector<double> obstacle = obstaclePtr->getObstacle();
                circular_obstacles.push_back(obstacle);
            }

            // Discretize map borders and add them to obstacle list
            std::vector<std::vector<double>> discretize_map_borders;

            for (std::vector<double> obstacle : mapBorder.discretizeBorder())
            {
                circular_obstacles.push_back(obstacle);
            }

            double final_orientation = M_PI / 2.0;
            final_orientation *= std::signbit(gate.position.y) ? -1 : 1;
            rrtstar::Node *goal = new rrtstar::Node(gate.position.x, gate.position.y, final_orientation);

            // TODO: loop for all shelfinos

            auto shelfino0 = shelfinos[0];
            rrtstar::Node *start = new rrtstar::Node(shelfino0.position.x, shelfino0.position.y, 0.0);

            // Create an instance of RRTStarDubins
            rrtstar::RRTStarDubins rrtStarDubins(start, goal, circular_obstacles, rndMin, rndMax);

            // Perform path planning
            auto start_time = std::chrono::high_resolution_clock::now();
            auto path = rrtStarDubins.planning(false);
            auto stop_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
            std::cout << "Path size: " << path.size() << " founded in: " << duration.count() << "milliseconds" << std::endl;

            // Iterate in reverse the path
            nav_msgs::msg::Path shelfino0_path;
            for (auto it = path.rbegin(); it != path.rend(); ++it)
            {
                const auto &point = *it;

                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.pose.position.x = point[0];
                poseStamped.pose.position.y = point[1];

                shelfino0_path.poses.push_back(poseStamped);
            }

            shelfino0PathPublisher_->publish(shelfino0_path);
        }
    }

    void dubins()
    {
        double start_x = 1.0;
        double start_y = 1.0;
        double start_yaw = M_PI / 4.0; // 45 degrees in radians

        double end_x = -3.0;
        double end_y = -3.0;
        double end_yaw = -M_PI / 4.0; // -45 degrees in radians

        double curvature = 1.0;

        std::cout << "starting dubins path \n";
        // Call the plan_dubins_path method
        auto [path_x, path_y, path_yaw, mode, lengths] = dubinsPath.plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature);
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