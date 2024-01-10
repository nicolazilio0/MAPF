#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <cmath>
#include <fstream>

#include "../include/CoordinateMapper.hpp"

#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "sensor_msgs/msg/image.hpp"

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

struct Victim
{
    double center_x;
    double center_y;
    double radius = 0.5;
    double value;
};

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class EnvironmentMap : public rclcpp::Node
{
public:
    EnvironmentMap()
        : Node("env_map"), coordinateMapper(20, 20, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        obstacles_subscritpion = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&EnvironmentMap::get_obstacles, this, _1));
        gates_subscritpion = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", qos, std::bind(&EnvironmentMap::get_gate, this, _1));
        map_subscritpion = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&EnvironmentMap::get_map, this, _1));
        victim_subscritpion = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("victims", qos, std::bind(&EnvironmentMap::get_victims, this, _1));

        map_publisher = this->create_publisher<sensor_msgs::msg::Image>("map_image", 10);

        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0_odom = std::bind(&EnvironmentMap::get_shelfino_position, this, _1, 0);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino1_odom = std::bind(&EnvironmentMap::get_shelfino_position, this, _1, 1);
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino2_odom = std::bind(&EnvironmentMap::get_shelfino_position, this, _1, 2);

        shelfino0_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0_odom);
        shelfino1_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino1/odom", 10, shelfino1_odom);
        shelfino2_subscritpion = this->create_subscription<nav_msgs::msg::Odometry>("shelfino2/odom", 10, shelfino2_odom);

        // --- Path visualizer ---
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino0_path = std::bind(&EnvironmentMap::get_path, this, _1, 0);
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino1_path = std::bind(&EnvironmentMap::get_path, this, _1, 1);
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino2_path = std::bind(&EnvironmentMap::get_path, this, _1, 2);

        shelfino0_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino0/plan1", 10, shelfino0_path);
        shelfino1_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino1/plan1", 10, shelfino1_path);
        shelfino2_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino2/plan1", 10, shelfino2_path);

        map_update_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EnvironmentMap::update_map, this));

        cv::namedWindow("Environment Map", cv::WINDOW_AUTOSIZE);
        map_image = cv::Mat(750, 750, CV_8UC3, cv::Scalar(255, 255, 255));
        map_image_copy = map_image.clone();
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscritpion;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_publisher;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victim_subscritpion;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino1_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino2_subscritpion;

    // --- Path visualizer ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0_path_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino1_path_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino2_path_subscritpion;

    cv::Mat map_image; // Declare mapImage as a class member
    cv::Mat map_image_copy;
    std::vector<Shelfino> shelfinos;
    std::vector<std::unique_ptr<Victim>> victims;

    Gate gate;

    CoordinateMapper coordinateMapper;
    rclcpp::TimerBase::SharedPtr map_update_timer;

    void draw_path(const nav_msgs::msg::Path::SharedPtr &msg, cv::Mat &image, const cv::Scalar &color = cv::Scalar(0, 0, 0))
    {

        for (size_t i = 0; i < msg->poses.size() - 1; ++i)
        {
            const auto &start = msg->poses[i].pose.position;
            const auto &end = msg->poses[i + 1].pose.position;

            int image1_x, image1_y, image2_x, image2_y;
            coordinateMapper.gazebo2img(start.x, start.y, image1_x, image1_y);
            coordinateMapper.gazebo2img(end.x, end.y, image2_x, image2_y);

            cv::line(image, cv::Point(image1_x, image1_y), cv::Point(image2_x, image2_y), color, 2);
        }
    }

    void draw_polygon(const geometry_msgs::msg::Polygon &polygon, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
    {
        // Convert polygon points to OpenCV points
        std::vector<cv::Point> cv_points;
        for (const auto &point : polygon.points)
        {
            int image_x, image_y;
            coordinateMapper.gazebo2img(point.x, point.y, image_x, image_y);
            cv_points.emplace_back(image_x, image_y);
        }

        // Draw polygon on the image
        if (fill)
        {
            cv::fillPoly(image, cv_points, color);
        }
        else
        {
            cv::polylines(image, cv_points, true, color, 2);
        }
    }

    void draw_circle(float radius, const geometry_msgs::msg::Point32 &center, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0), bool draw_text = false, int text_value = 0)
    {
        int image_center_x, image_center_y;
        coordinateMapper.gazebo2img(center.x, center.y, image_center_x, image_center_y);

        int image_radius;
        coordinateMapper.gazebo2img(radius, image_radius);

        int thickness = fill ? -1 : 2;

        // Draw circle on the image

        cv::circle(image, cv::Point(image_center_x, image_center_y), static_cast<int>(image_radius), color, thickness);
        if (draw_text)
        {
            // Draw text at the center of the circle
            std::string text = std::to_string(text_value);
            int font = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.5;
            int font_thickness = 1.5;
            int baseline = 0;

            cv::Size textSize = cv::getTextSize(text, font, font_scale, font_thickness, &baseline);

            int text_x = image_center_x - textSize.width / 2;
            int text_y = image_center_y + textSize.height / 2;

            cv::putText(image, text, cv::Point(text_x, text_y), font, font_scale, cv::Scalar(255, 255, 255), font_thickness);
        }
    }

    void draw_rectangle(double width, double height, const geometry_msgs::msg::Point &center, const geometry_msgs::msg::Quaternion &orientation, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
    {
        // Convert quaternion to rotation matrix
        Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

        double half_width = width / 2.0;
        double half_height = height / 2.0;

        Eigen::Vector3d corners[4] = {
            {-half_width, half_height, 0},
            {half_width, half_height, 0},
            {half_width, -half_height, 0},
            {-half_width, -half_height, 0}};

        // Transform the object space points to world space and convert to OpenCV points
        std::vector<cv::Point> cv_points;
        cv_points.reserve(4);
        for (const auto &corner : corners)
        {
            Eigen::Vector3d transformed_corner = (rotation_matrix * corner).eval();
            transformed_corner.x() += center.x;
            transformed_corner.y() += center.y;

            int image_x, image_y;
            coordinateMapper.gazebo2img(transformed_corner.x(), transformed_corner.y(), image_x, image_y);
            cv_points.emplace_back(image_x, image_y);
        }

        // Draw square on the image
        if (fill)
        {
            cv::fillConvexPoly(image, cv_points.data(), 4, color);
        }
        else
        {
            cv::polylines(image, cv_points, true, color, 2);
        }
    }

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
                this->draw_polygon(polygon, map_image, true, cv::Scalar(0, 255, 0));
            }
            else
            {
                // Circle only contains one polygon
                const auto &point = polygon.points[0];
                this->draw_circle(radius, point, map_image, true, cv::Scalar(0, 255, 0));
            }
        }

        sensor_msgs::msg::Image::SharedPtr img_msg;
        img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", map_image).toImageMsg();
        map_publisher->publish(*img_msg.get());
    }

    void get_gate(const geometry_msgs::msg::PoseArray &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received gate");

        const auto &pose = msg.poses[0];
        gate.position = pose.position;
        gate.orientation = pose.orientation;

        this->draw_rectangle(1.0, 1.0, gate.position, geometry_msgs::msg::Quaternion(), map_image, true, cv::Scalar(0, 0, 255));
    }

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
            double radius = 0.5;
            this->draw_circle(radius, point, map_image, true, cv::Scalar(255, 0, 0), true, value);
        }

        RCLCPP_INFO(this->get_logger(), "Received victims");
    }

    void get_map(const geometry_msgs::msg::Polygon &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map borders");

        this->draw_polygon(msg, map_image, false);
    }

    void get_shelfino_position(const nav_msgs::msg::Odometry::SharedPtr &msg, int robot_id)
    {

        // Find the RobotInfo for the specified robot_id
        auto it = std::find_if(shelfinos.begin(), shelfinos.end(),
                               [robot_id](const Shelfino &shelfino)
                               { return shelfino.id == robot_id; });

        if (it == shelfinos.end())
        {
            Shelfino shelfino;
            shelfino.id = robot_id;
            shelfinos.push_back(shelfino);
            it = std::prev(shelfinos.end());
        }

        // Update the position and orientation of the robot
        it->position = msg->pose.pose.position;
        it->orientation = msg->pose.pose.orientation;
    }

    void get_path(const nav_msgs::msg::Path::SharedPtr &msg, int robot_id)
    {
        RCLCPP_INFO(this->get_logger(), "Received Path for Shelfino %i", robot_id);

        int color = 50 + (50 * robot_id);

        this->draw_path(msg, map_image, cv::Scalar(color, color, color));
    }

    void update_map()
    {
        map_image_copy = map_image.clone();

        for (const auto &shelfino : shelfinos)
        {
            int color = 50 + (50 * shelfino.id);
            this->draw_rectangle(.5, .5, shelfino.position, shelfino.orientation, map_image_copy, true, cv::Scalar(color, color, color));
        }

        // Plot the map
        cv::imshow("Environment Map", map_image_copy);
        cv::waitKey(1); // Non-blocking wait key
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto environmentMap = std::make_shared<EnvironmentMap>();
    rclcpp::spin(environmentMap);
    rclcpp::shutdown();
    return 0;
}