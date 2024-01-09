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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class EnvironmentMap : public rclcpp::Node
{
public:
  EnvironmentMap()
      : Node("env_map"), coordinateMapper(20, 20, 750, 750)
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&EnvironmentMap::getObstacles, this, _1));
    gatesSubscritpion_ = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", qos, std::bind(&EnvironmentMap::getGates, this, _1));
    mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&EnvironmentMap::getMap, this, _1));

    mapPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("map_image", 10);

    std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0Odom = std::bind(&EnvironmentMap::getShelfinoPosition, this, _1, 0);
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino1Odom = std::bind(&EnvironmentMap::getShelfinoPosition, this, _1, 1);
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino2Odom = std::bind(&EnvironmentMap::getShelfinoPosition, this, _1, 2);

    shelfino0Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0Odom);
    shelfino1Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino1/odom", 10, shelfino1Odom);
    shelfino2Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino2/odom", 10, shelfino2Odom);

    // --- Voronoi visualizer ---
    voronoiSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("voronoi", 10, std::bind(&EnvironmentMap::getVoronoi, this, _1));

    // --- Path visualizer ---
    std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino0Path = std::bind(&EnvironmentMap::getPath, this, _1, 0);
    std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino1Path = std::bind(&EnvironmentMap::getPath, this, _1, 1);
    std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino2Path = std::bind(&EnvironmentMap::getPath, this, _1, 2);

    shelfino0PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino0/plan1", 10, shelfino0Path);
    shelfino1PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino1/plan1", 10, shelfino1Path);
    shelfino2PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino2/plan1", 10, shelfino2Path);

    mapUpdateTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EnvironmentMap::updateMap, this));

    cv::namedWindow("Environment Map", cv::WINDOW_AUTOSIZE);
    mapImage = cv::Mat(750, 750, CV_8UC3, cv::Scalar(255, 255, 255));
    mapImageCopy = mapImage.clone();
  }

private:
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mapPublisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0Subscritpion_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino1Subscritpion_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino2Subscritpion_;

  // --- Voronoi visualizer ---
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr voronoiSubscritpion_;

  // --- Path visualizer ---
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0PathSubscritpion_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino1PathSubscritpion_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino2PathSubscritpion_;

  cv::Mat mapImage; // Declare mapImage as a class member
  cv::Mat mapImageCopy;
  std::vector<Shelfino> shelfinos;
  Gate gate;

  CoordinateMapper coordinateMapper;
  rclcpp::TimerBase::SharedPtr mapUpdateTimer_;

  void drawVoronoi(const geometry_msgs::msg::Polygon &polygon, cv::Mat &image, const cv::Scalar &color = cv::Scalar(2, 134, 242))
  {
    for (size_t i = 0; i < polygon.points.size(); i++)
    {
      int image1X, image1Y;
      auto point = polygon.points[i];
      coordinateMapper.gazebo2img(point.x, point.y, image1X, image1Y);

      cv::circle(image, cv::Point(image1X, image1Y), 2, color, 2);
    }
  }

  void drawPath(const nav_msgs::msg::Path::SharedPtr &msg, cv::Mat &image, const cv::Scalar &color = cv::Scalar(0, 0, 0))
  {

    for (size_t i = 0; i < msg->poses.size() - 1; ++i)
    {
      const auto &start = msg->poses[i].pose.position;
      const auto &end = msg->poses[i + 1].pose.position;

      int image1X, image1Y, image2X, image2Y;
      coordinateMapper.gazebo2img(start.x, start.y, image1X, image1Y);
      coordinateMapper.gazebo2img(end.x, end.y, image2X, image2Y);

      cv::line(image, cv::Point(image1X, image1Y), cv::Point(image2X, image2Y), color, 2);
    }
  }

  void drawPolygon(const geometry_msgs::msg::Polygon &polygon, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
  {
    // Convert polygon points to OpenCV points
    std::vector<cv::Point> cvPoints;
    for (const auto &point : polygon.points)
    {
      int imageX, imageY;
      // fix flipped coordinates
      // TODO: maybe integrate this in the coordinateclass
      coordinateMapper.gazebo2img(point.x, point.y, imageX, imageY);
      cvPoints.emplace_back(imageX, imageY);
    }

    // Draw polygon on the image
    if (fill)
    {
      cv::fillPoly(image, cvPoints, color);
    }
    else
    {
      cv::polylines(image, cvPoints, true, color, 2);
    }
  }

  void drawCircle(float radius, const geometry_msgs::msg::Point32 &center, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
  {
    int imageCenterX, imageCenterY;
    // fix flipped coordinates
    // TODO: maybe integrate this in the coordinateclass
    coordinateMapper.gazebo2img(center.x, center.y, imageCenterX, imageCenterY);

    int imageRadius;
    coordinateMapper.gazebo2img(radius, imageRadius);

    int thickness = fill ? -1 : 2;

    // Draw circle on the image

    cv::circle(image, cv::Point(imageCenterX, imageCenterY), static_cast<int>(imageRadius), color, thickness);
  }

  void drawRectangle(double width, double height, const geometry_msgs::msg::Point &center, const geometry_msgs::msg::Quaternion &orientation, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
  {
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    double halfWidth = width / 2.0;
    double halfHeight = height / 2.0;

    Eigen::Vector3d corners[4] = {
        {-halfWidth, halfHeight, 0},
        {halfWidth, halfHeight, 0},
        {halfWidth, -halfHeight, 0},
        {-halfWidth, -halfHeight, 0}};

    // Transform the object space points to world space and convert to OpenCV points
    std::vector<cv::Point> cvPoints;
    cvPoints.reserve(4);
    for (const auto &corner : corners)
    {
      Eigen::Vector3d transformedCorner = (rotationMatrix * corner).eval();
      transformedCorner.x() += center.x;
      transformedCorner.y() += center.y;

      int imageX, imageY;
      coordinateMapper.gazebo2img(transformedCorner.x(), transformedCorner.y(), imageX, imageY);
      cvPoints.emplace_back(imageX, imageY);
    }

    // Draw square on the image
    if (fill)
    {
      cv::fillConvexPoly(image, cvPoints.data(), 4, color);
    }
    else
    {
      cv::polylines(image, cvPoints, true, color, 2);
    }
  }

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
        this->drawPolygon(polygon, mapImage, true, cv::Scalar(0, 255, 0));
      }
      else
      {
        // Circle only contains one polygon
        const auto &point = polygon.points[0];
        this->drawCircle(radius, point, mapImage, true, cv::Scalar(0, 255, 0));
      }
    }

    sensor_msgs::msg::Image::SharedPtr imgMsg;
    imgMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mapImage).toImageMsg();
    mapPublisher_->publish(*imgMsg.get());
  }

  void getGates(const geometry_msgs::msg::PoseArray &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received gate");

    const auto &pose = msg.poses[0];
    gate.position = pose.position;
    gate.orientation = pose.orientation;

    this->drawRectangle(1.0, 1.0, gate.position, geometry_msgs::msg::Quaternion(), mapImage, true, cv::Scalar(0, 0, 255));
  }

  void getMap(const geometry_msgs::msg::Polygon &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map borders");

    this->drawPolygon(msg, mapImage, false);
  }

  void getShelfinoPosition(const nav_msgs::msg::Odometry::SharedPtr &msg, int robotId)
  {

    // Find the RobotInfo for the specified robotId
    auto it = std::find_if(shelfinos.begin(), shelfinos.end(),
                           [robotId](const Shelfino &shelfino)
                           { return shelfino.id == robotId; });

    if (it == shelfinos.end())
    {
      Shelfino shelfino;
      shelfino.id = robotId;
      shelfinos.push_back(shelfino);
      it = std::prev(shelfinos.end());
    }

    // Update the position and orientation of the robot
    it->position = msg->pose.pose.position;
    it->orientation = msg->pose.pose.orientation;
  }

  void getVoronoi(const geometry_msgs::msg::Polygon &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received voroni map");

    // Draw map borders
    this->drawVoronoi(msg, mapImage);
  }

  void getPath(const nav_msgs::msg::Path::SharedPtr &msg, int robotId)
  {
    RCLCPP_INFO(this->get_logger(), "Received Path for Shelfino %i", robotId);

    int colorIdentifier = 50 + (50 * robotId);

    this->drawPath(msg, mapImage, cv::Scalar(colorIdentifier, colorIdentifier, colorIdentifier));
  }

  void updateMap()
  {
    mapImageCopy = mapImage.clone();

    for (const auto &shelfino : shelfinos)
    {
      int colorIdentifier = 50 + (50 * shelfino.id);
      this->drawRectangle(.5, .5, shelfino.position, shelfino.orientation, mapImageCopy, true, cv::Scalar(colorIdentifier, colorIdentifier, colorIdentifier));
    }

    // Plot the map
    cv::imshow("Environment Map", mapImageCopy);
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