#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

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

#include "CoordinateMapper.hpp"
#include "DubinsPath.hpp"

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

class RRTStarDubinsPlanner : public rclcpp::Node
{
public:
    RRTStarDubinsPlanner()
        : Node("rrtstardubins_planner"), coordinateMapper(17, 17, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
        dubinsTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RRTStarDubinsPlanner::dubins, this));
    }

private:
    CoordinateMapper coordinateMapper;
    DubinsPath dubinsPath;
    rclcpp::TimerBase::SharedPtr dubinsTimer_;

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

        std::cout << "X -------------" << std::endl;

        for (const auto &x : path_x)
        {
            std::cout << x << " , ";
        }

        std::cout << std::endl
                  << "Y -------------" << std::endl;

        for (const auto &y : path_y)
        {
            std::cout << y << " , ";
        }

        std::cout << std::endl
                  << "Yaw -------------" << std::endl;

        for (const auto &yaw : path_yaw)
        {
            std::cout << yaw << " , ";
        }

        std::cout << std::endl
                  << "******************" << std::endl;

        // // Log results using ROS 2 logging
        // RCLCPP_INFO(this->get_logger(), "Dubins Path Planning Results:");
        // // RCLCPP_INFO(this->get_logger(), "Mode: %s", mode.c_str());

        // RCLCPP_INFO(this->get_logger(), "Path X:");
        // for (const auto &x : path_x)
        // {
        //     RCLCPP_INFO(this->get_logger(), "%.2f", x);
        // }

        // RCLCPP_INFO(this->get_logger(), "Path Y:");
        // for (const auto &y : path_y)
        // {
        //     RCLCPP_INFO(this->get_logger(), "%.2f", y);
        // }

        // RCLCPP_INFO(this->get_logger(), "Path Yaw:");
        // for (const auto &yaw : path_yaw)
        // {
        //     RCLCPP_INFO(this->get_logger(), "%.2f", yaw);
        // }

        // RCLCPP_INFO(this->get_logger(), "Lengths:");
        // for (const auto &len : lengths)
        // {
        //     RCLCPP_INFO(this->get_logger(), "%.2f", len);
        // }
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