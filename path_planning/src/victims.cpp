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

struct Victim
{
    double center_x;
    double center_y;
    double radius = 0.5;
    double value;
};

struct MapBorder
{
    geometry_msgs::msg::Polygon polygon;

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
        victim_subscritpion = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("victims", qos, std::bind(&RRTStarDubinsPlanner::get_victims, this, _1));

        shelfino0_path_publisher = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);
        shelfino0_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, std::bind(&RRTStarDubinsPlanner::get_shelfino_position, this, _1));

        obstacles_aquired = false;
        shelfino_aquired = false;
        map_aquired = false;
        gate_aquired = false;
        victims_aquired = false;

        rnd_min = -10;
        rnd_max = 10;

        planner_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RRTStarDubinsPlanner::generate_roadmap, this));
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscritpion;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victim_subscritpion;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0_subscritpion;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino0_path_publisher;

    MapBorder mapBorder;
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    std::vector<std::unique_ptr<Victim>> victims;
    std::vector<std::unique_ptr<Victim>> victims_copy;

    Shelfino shelfino0;
    Gate gate;

    CoordinateMapper coordinateMapper;
    DubinsPath dubinsPath;
    rclcpp::TimerBase::SharedPtr planner_timer;

    bool map_aquired;
    bool obstacles_aquired;
    bool gate_aquired;
    bool shelfino_aquired;
    bool victims_aquired;

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

    void get_shelfino_position(const nav_msgs::msg::Odometry &msg)
    {

        shelfino0.id = 0;

        shelfino0.position = msg.pose.pose.position;
        shelfino0.orientation = msg.pose.pose.orientation;

        shelfino0_subscritpion.reset();

        RCLCPP_INFO(this->get_logger(), "Recived all Shelfinos");

        shelfino_aquired = true;
    }

    // Callbacks for topic handling
    void get_victims(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received %zu victims", msg.obstacles.size());
        for (size_t i = 0; i < msg.obstacles.size(); i++)
        {
            const auto &msg_victims = msg.obstacles[i];

            // Access the polygon and radius fields for each obstacle
            const auto &polygon = msg_victims.polygon;
            const auto &value = msg_victims.radius;

            // Circle only contains one polygon
            const auto &point = polygon.points[0];
            auto victim = std::make_unique<Victim>();
            victim->center_x = point.x;
            victim->center_y = point.y;
            victim->value = value;
            victims.push_back(std::move(victim));
            victims_copy.push_back(std::move(victim));
        }

        victims_aquired = true;
    }

    // recursive function to try to insert other victims in the path
    std::vector<std::vector<double>> calculate_path_recursive(rrtstar::Node *vicrim_pred, rrtstar::Node *goal, std::vector<std::vector<double>> circular_obstacles, double residual_time, double value, std::vector<std::vector<double>> path, double rnd_max, double rnd_min)
    {
        std::vector<std::vector<double>> path_to_victim;
        std::vector<std::vector<double>> best_path = path;

        // loops on all the victims
        for (const auto &victim : victims)
        {
            std::vector<std::vector<double>> tmp_path = path;

            RCLCPP_INFO(this->get_logger(), "Calculating RRT* for victim %f", victim->value);

            // while victims copy is not empty and the victim is not in the copy (it means that the victim has already been inserted in the path)
            if (!victims_copy.empty())
            {
                rrtstar::Node *victim_next = new rrtstar::Node(victim->center_x, victim->center_y, 0.785);
                rrtstar::RRTStarDubins rrtStarDubins(vicrim_pred, victim_next, circular_obstacles, rnd_min, rnd_max);

                // calculate path from victim to goal
                int counter = 0;

                do
                {
                    path_to_victim = rrtStarDubins.planning(false);
                    counter = counter + 1;
                    if (path_to_victim.empty() && counter >= 1)
                    {
                        break;
                    }
                } while (path_to_victim.empty());

                if (path_to_victim.empty())
                {
                    continue;
                }
                // calculates the base path to the goal
                std::vector<std::vector<double>> path_victim_to_goal = calculate_path_from_victims_to_goal(goal, victim_next, circular_obstacles, rnd_max, rnd_min);

                auto path_duration = path_to_victim.size() * 0.1 / 0.2;
                auto path_victim_to_goal_duration = path_victim_to_goal.size() * 0.1 / 0.2;
                residual_time = residual_time - path_duration - path_victim_to_goal_duration;

                // for debug
                // RCLCPP_ERROR(this->get_logger(), "Residual time %f", residual_time);
                // if we don't have time then the victim cannot be inserted
                if (residual_time < 0)
                {
                    // RCLCPP_ERROR(this->get_logger(), "Victim %f cannot be reached in time", victim->value);
                    continue;
                }
                else
                {

                    // for debug
                    //  RCLCPP_INFO(this->get_logger(), "Victim %f can be reached in time", victim->value);
                    //  RCLCPP_ERROR(this->get_logger(), "Time remaining %f ", residual_time);
                    double value_now = victim->value + value;
                    value = std::max(value_now, value);

                    if (value == value_now)
                    {
                        RCLCPP_INFO(this->get_logger(), "BEST VALUE");
                        // pushes back the path to save the best path
                        std::reverse(path_to_victim.begin(), path_to_victim.end());
                        tmp_path.insert(tmp_path.end(), path_to_victim.begin(), path_to_victim.end());
                        tmp_path.insert(tmp_path.end(), path_victim_to_goal.begin(), path_victim_to_goal.end());
                        std::reverse(tmp_path.begin(), tmp_path.end());
                        best_path = tmp_path;
                        continue;
                    }
                    else
                    {
                        // we did not have an improvement in the value
                        RCLCPP_INFO(this->get_logger(), "NOT BEST VALUE");
                        continue;
                    }
                }
                // removes the victim we are taking into consideration from the copy
                std::vector<std::unique_ptr<Victim>>::iterator iter = std::find(victims_copy.begin(), victims_copy.end(), victim);
                if (iter != victims_copy.end())
                {
                    victims_copy.erase(iter);
                    // RCLCPP_INFO(this->get_logger(), "Trying recursive insertion, victims_size %zu", victims_copy.size());
                    continue;
                }
            }
            else
            {
                // inserts back the victim we are removing to return to a previous state
                RCLCPP_INFO(this->get_logger(), "ELSE IN THE END");
                std::vector<std::unique_ptr<Victim>>::iterator asd = std::find(victims.begin(), victims.end(), victim);
                if (asd != victims.end())
                {
                    victims_copy.push_back(std::move(*asd));
                }

                continue;
            }
            // pushes back the victim we are taking into consideration from the copy
            std::vector<std::unique_ptr<Victim>>::iterator ti = std::find(victims.begin(), victims.end(), victim);
            if (ti != victims.end())
            {
                victims_copy.push_back(std::move(*ti));
            }
        }

        return best_path;
    }

    // base case of path calculation
    // calculate path from start to victim and from victim to goal
    // calls recursive function to try to insert other victims in the path
    std::vector<std::vector<double>> calculate_path_base(rrtstar::Node *start, rrtstar::Node *goal, std::vector<std::vector<double>> circular_obstacles, double residual_time, double rnd_max, double rnd_min)
    {
        std::vector<std::vector<double>> path_to_goal;
        std::vector<std::vector<double>> actual_path;
        double max_value = 0;
        std::vector<std::vector<double>> best_path;
        double remaining_time = residual_time;
        if (remaining_time < 0)
        {
            return best_path;
        }
        else
        {
            for (const auto &victim : victims)
            {
                // calculate path from start to victim
                rrtstar::Node *victim_goal = new rrtstar::Node(victim->center_x, victim->center_y, 0.785);
                rrtstar::Node *victim_exit = new rrtstar::Node(victim->center_x, victim->center_y, 0.785);
                rrtstar::RRTStarDubins rrtStarDubins(start, victim_goal, circular_obstacles, rnd_min, rnd_max);
                int count = 0;
                do
                {
                    path_to_goal = rrtStarDubins.planning(false);
                    count = count + 1;
                    if (path_to_goal.empty() && count >= 1)
                    {
                        break;
                    }
                } while (path_to_goal.empty() || count < 1);

                if (path_to_goal.empty())
                {
                    continue;
                }

                // calculates the base path to the goal
                std::vector<std::vector<double>> path_victim_to_goal = calculate_path_from_victims_to_goal(goal, victim_exit, circular_obstacles, rnd_max, rnd_min);

                std::vector<std::unique_ptr<Victim>>::iterator it = std::find(victims_copy.begin(), victims_copy.end(), victim);
                if (it != victims_copy.end())
                    victims_copy.erase(it);

                // RCLCPP_INFO(this->get_logger(), "Trying recursive insertion, victims_size %zu", victims_copy.size());

                auto path_duration = path_to_goal.size() * 0.1 / 0.2;
                auto path_victim_to_goal_duration = path_victim_to_goal.size() * 0.1 / 0.2;
                remaining_time = remaining_time - path_duration - path_victim_to_goal_duration;
                std::reverse(path_to_goal.begin(), path_to_goal.end());
                // if we still have time left we try to insert other victims
                if (remaining_time > 0)
                {
                    double value = victim->value;

                    // keeps track of best victims value
                    max_value = std::max(max_value, value);

                    // recursive call to try to insert other victims
                    auto path_to_goal_recursive = calculate_path_recursive(victim_goal, goal, circular_obstacles, remaining_time, value, path_to_goal, rnd_max, rnd_min);
                    // keeps track of better value
                    if (max_value == value)
                    {
                        RCLCPP_INFO(this->get_logger(), "EXIT RECURSION IF");
                        best_path = path_to_goal_recursive;
                        return best_path;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "EXIT RECURSION ELSE");
                    }

                    // for debug
                    // RCLCPP_INFO(this->get_logger(), "Victim %f can be reached in time", victim->value);
                    // RCLCPP_INFO(this->get_logger(), "Time remaining %f ", remaining_time);
                }
                else
                {
                    // for debug
                    // RCLCPP_INFO(this->get_logger(), "Victim %f cannot be reached in time", victim->value);
                    continue;
                }
            }
            if (best_path.empty())
            {
                // if we cannot insert other victims we calculate the path from start to goal
                std::vector<std::vector<double>> path_start_to_goal = calculate_path_from_victims_to_goal(goal, start, circular_obstacles, rnd_max, rnd_min);
                return path_start_to_goal;
            }
            else
            {
                return best_path;
            }
        }
    }

    // Function for path computing from victims to goal
    std::vector<std::vector<double>> calculate_path_from_victims_to_goal(rrtstar::Node *goal, rrtstar::Node *victim_node, std::vector<std::vector<double>> circular_obstacles, double rnd_max, double rnd_min)
    {

        std::vector<std::vector<double>> path_to_goal;
        // computes tha path from the designated victim to the goal position
        rrtstar::RRTStarDubins rrtStarDubins(victim_node, goal, circular_obstacles, rnd_min, rnd_max);
        do
        {
            path_to_goal = rrtStarDubins.planning(false);
        } while (path_to_goal.empty());
        std::reverse(path_to_goal.begin(), path_to_goal.end());
        return path_to_goal;
    }

    void generate_roadmap()
    {
        if (obstacles_aquired && gate_aquired && shelfino_aquired && map_aquired && victims_aquired)
        {
            RCLCPP_INFO(this->get_logger(), "Executing RRT*");
            double max_time = 1000.0; // [s]
            // Stop the timer callback
            planner_timer->cancel();

            // Add obstacles to the obstacle list
            std::vector<std::vector<double>> circular_obstacles;

            for (const auto &obstaclePtr : obstacles)
            {
                std::vector<double> obstacle = obstaclePtr->get_obstacle();
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

            auto id = shelfino0.id;
            auto shelfino_position = shelfino0.position;
            auto shelfino_orientation = shelfino0.orientation;

            // Get shelfino's yaw angle
            double t0 = 2.0 * (shelfino_orientation.w * shelfino_orientation.z + shelfino_orientation.x * shelfino_orientation.y);
            double t1 = 1.0 - 2.0 * (shelfino_orientation.y * shelfino_orientation.y + shelfino_orientation.z * shelfino_orientation.z);
            double yaw = std::atan2(t0, t1);

            rrtstar::Node *start = new rrtstar::Node(shelfino_position.x, shelfino_position.y, yaw);

            auto start_time = std::chrono::high_resolution_clock::now();

            // Calculate first path from start position to victim for each victim
            std::vector<std::vector<double>> path = calculate_path_base(start, goal, circular_obstacles, max_time, rnd_max, rnd_min);

            RCLCPP_INFO(this->get_logger(), "Path size: %zu", path.size());

            auto stop_time = std::chrono::high_resolution_clock::now();
            auto duration = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count()) / 1000.0;
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

            // Sent the path accordingly to shelfino 0

            shelfino0_path_publisher->publish(shelfino_path);
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