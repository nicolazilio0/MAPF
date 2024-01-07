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

#include "../include/CoordinateMapper.hpp"

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

struct Obstacle
{
    virtual geometry_msgs::msg::Polygon get_polygon() const = 0;
};

struct PolygonObstacle : Obstacle
{
    geometry_msgs::msg::Polygon polygon;

    geometry_msgs::msg::Polygon get_polygon() const override
    {
        return polygon;
    }
};

struct CylinderObstacle : Obstacle
{
    double center_x;
    double center_y;
    double radius;
    double num_segments = 10;

    geometry_msgs::msg::Polygon get_polygon() const override
    {
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
        map_subscritpion = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&VoronoiPlanner::get_map, this, _1));
        voronoi_publisher = this->create_publisher<geometry_msgs::msg::Polygon>("voronoi", 10);
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscritpion;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscritpion;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr voronoi_publisher;

    std::vector<std::unique_ptr<Obstacle>> obstacles;
    geometry_msgs::msg::Polygon map_borders;

    CoordinateMapper coordinateMapper;

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
                obstacle->radius = radius / 2.0;
                obstacles.push_back(std::move(obstacle));
            }
        }

        generateVoronoiDiagram();
    }

    void get_map(const geometry_msgs::msg::Polygon &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map borders");

        map_borders = msg;
    }

    void generateVoronoiDiagram()
    {

        std::vector<Segment> segments;
        int image1_x, image1_y, image2_x, image2_y;

        // Convert obstacles and map borders
        for (const auto &obstacle : obstacles)
        {
            const auto &ros_polygon = obstacle->get_polygon();
            RCLCPP_INFO(this->get_logger(), "-- Obstacle --");

            for (size_t i = 0; i < ros_polygon.points.size() - 1; ++i)
            {
                const auto &point1 = ros_polygon.points[i];
                const auto &point2 = ros_polygon.points[i + 1];
                coordinateMapper.gazebo2img(point1.x, point1.y, image1_x, image1_y);
                coordinateMapper.gazebo2img(point2.x, point2.y, image2_x, image2_y);
                RCLCPP_INFO(this->get_logger(), "%i,%i ; %i,%i", image1_x, image1_y, image2_x, image2_y);

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
        RCLCPP_INFO(this->get_logger(), "-- Map --");

        for (size_t i = 0; i < map_borders.points.size() - 1; ++i)
        {
            const auto &point1 = map_borders.points[i];
            const auto &point2 = map_borders.points[i + 1];
            coordinateMapper.gazebo2img(point1.x, point1.y, image1_x, image1_y);
            coordinateMapper.gazebo2img(point2.x, point2.y, image2_x, image2_y);
            RCLCPP_INFO(this->get_logger(), "%i,%i ; %i,%i", image1_x, image1_y, image2_x, image2_y);

            segments.push_back(Segment(image1_x, image1_y, image2_x, image2_y));
        }

        // Connect the last point of map borders with the first one
        const auto &first_map_point = map_borders.points.front();
        const auto &last_map_point = map_borders.points.back();
        coordinateMapper.gazebo2img(first_map_point.x, first_map_point.y, image1_x, image1_y);
        coordinateMapper.gazebo2img(last_map_point.x, last_map_point.y, image2_x, image2_y);

        segments.push_back(Segment(image1_x, image1_y, image2_x, image2_y));

        boost::polygon::voronoi_diagram<double> vd;
        boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

        geometry_msgs::msg::Polygon voronoi_polygon;

        for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
        {
            if (it->is_finite() && it->is_primary())
            {
                // Edge start point
                geometry_msgs::msg::Point32 point;
                point.x = static_cast<int>(it->vertex0()->x());
                point.y = static_cast<int>(it->vertex0()->y());
                voronoi_polygon.points.push_back(point);

                // Edge end point
                point.x = static_cast<int>(it->vertex1()->x());
                point.y = static_cast<int>(it->vertex1()->y());
                voronoi_polygon.points.push_back(point);
            }
        }

        voronoi_publisher->publish(voronoi_polygon);
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