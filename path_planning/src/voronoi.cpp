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

#include <boost/polygon/voronoi.hpp>

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

struct Obstacle
{
    virtual geometry_msgs::msg::Polygon getPolygon() const = 0;
};

struct PolygonObstacle : Obstacle
{
    geometry_msgs::msg::Polygon polygon;

    geometry_msgs::msg::Polygon getPolygon() const override
    {
        return polygon;
    }
};

struct CylinderObstacle : Obstacle
{
    double centerX;
    double centerY;
    double radius;
    double numSegments = 10;

    geometry_msgs::msg::Polygon getPolygon() const override
    {
        geometry_msgs::msg::Polygon polygon;

        for (int i = 0; i < numSegments; i++)
        {
            double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(numSegments);
            double x = centerX + radius * std::cos(theta);
            double y = centerY + radius * std::sin(theta);

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
        : Node("vornoi_planner"), coordinateMapper(17, 17, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&VoronoiPlanner::getObstacles, this, _1));
        mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&VoronoiPlanner::getMap, this, _1));
        voronoiPublisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("voronoi", 10);
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr voronoiPublisher_;

    std::vector<std::unique_ptr<Obstacle>> obstacles;
    geometry_msgs::msg::Polygon mapBorders;

    CoordinateMapper coordinateMapper;

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

        generateVoronoiDiagram();
    }

    void getMap(const geometry_msgs::msg::Polygon &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map borders");

        mapBorders = msg;
    }

    void generateVoronoiDiagram()
    {

        std::vector<Segment> segments;
        int image1X, image1Y, image2X, image2Y;

        // Convert obstacles and map borders
        for (const auto &obstacle : obstacles)
        {
            const auto &rosPolygon = obstacle->getPolygon();
            RCLCPP_INFO(this->get_logger(), "-- Obstacle --");

            for (size_t i = 0; i < rosPolygon.points.size() - 1; ++i)
            {
                const auto &point1 = rosPolygon.points[i];
                const auto &point2 = rosPolygon.points[i + 1];
                coordinateMapper.gazebo2img(point1.x, point1.y, image1X, image1Y);
                coordinateMapper.gazebo2img(point2.x, point2.y, image2X, image2Y);
                RCLCPP_INFO(this->get_logger(), "%i,%i ; %i,%i", image1X, image1Y, image2X, image2Y);

                segments.push_back(Segment(image1X, image1Y, image2X, image2Y));
            }

            // Connect the last point with the first one
            const auto &firstPoint = rosPolygon.points.front();
            const auto &lastPoint = rosPolygon.points.back();
            coordinateMapper.gazebo2img(firstPoint.x, firstPoint.y, image1X, image1Y);
            coordinateMapper.gazebo2img(lastPoint.x, lastPoint.y, image2X, image2Y);

            segments.push_back(Segment(image1X, image1Y, image2X, image2Y));
        }

        // Create segments for map borders
        RCLCPP_INFO(this->get_logger(), "-- Map --");

        for (size_t i = 0; i < mapBorders.points.size() - 1; ++i)
        {
            const auto &point1 = mapBorders.points[i];
            const auto &point2 = mapBorders.points[i + 1];
            coordinateMapper.gazebo2img(point1.x, point1.y, image1X, image1Y);
            coordinateMapper.gazebo2img(point2.x, point2.y, image2X, image2Y);
            RCLCPP_INFO(this->get_logger(), "%i,%i ; %i,%i", image1X, image1Y, image2X, image2Y);

            segments.push_back(Segment(image1X, image1Y, image2X, image2Y));
        }

        // Connect the last point of map borders with the first one
        const auto &firstMapPoint = mapBorders.points.front();
        const auto &lastMapPoint = mapBorders.points.back();
        coordinateMapper.gazebo2img(firstMapPoint.x, firstMapPoint.y, image1X, image1Y);
        coordinateMapper.gazebo2img(lastMapPoint.x, lastMapPoint.y, image2X, image2Y);

        segments.push_back(Segment(image1X, image1Y, image2X, image2Y));

        boost::polygon::voronoi_diagram<double> vd;
        boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

        geometry_msgs::msg::Polygon voronoiPolygon;

        for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
        {
            if (it->is_finite() && it->is_primary())
            {
                // Edge start point
                geometry_msgs::msg::Point32 point;
                point.x = static_cast<int>(it->vertex0()->x());
                point.y = static_cast<int>(it->vertex0()->y());
                voronoiPolygon.points.push_back(point);

                // Edge end point
                point.x = static_cast<int>(it->vertex1()->x());
                point.y = static_cast<int>(it->vertex1()->y());
                voronoiPolygon.points.push_back(point);
            }
        }

        // for (auto it = vd.vertices().begin(); it != vd.vertices().end(); ++it)
        // {
        //     const auto &vertex = *it;
        //     // RCLCPP_INFO(this->get_logger(), "Voronoi vertex: %f, %f", vertex.x(), vertex.y());

        //     // Create a geometry_msgs::msg::Point message and add it to the polygon
        //     geometry_msgs::msg::Point32 point;
        //     point.x = vertex.x();
        //     point.y = vertex.y();
        //     point.z = 0.0; // Assuming 2D, so set z to 0

        //     voronoiPolygon.points.push_back(point);
        // }
        // Publish the geometry_msgs::msg::Polygon
        voronoiPublisher_->publish(voronoiPolygon);
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