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

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

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

using std::placeholders::_1;

class CoordinateMapper
{
public:
  CoordinateMapper(double realWorldWidth, double realWorldHeight, double imageWidth, double imageHeight)
      : realWorldWidth_(realWorldWidth), realWorldHeight_(realWorldHeight),
        imageWidth_(imageWidth), imageHeight_(imageHeight)
  {
    // Calculate scaling factors
    scaleX_ = imageWidth_ / realWorldWidth_;
    scaleY_ = imageHeight_ / realWorldHeight_;

    // Calculate offset
    offsetX_ = imageWidth_ / 2.0;
    offsetY_ = imageHeight_ / 2.0;
  }

  void convertToImageCoordinates(double realX, double realY, double &imageX, double &imageY) const
  {
    imageX = std::round(realX * scaleX_ + offsetX_);
    imageY = std::round(realY * scaleY_ + offsetY_);
  }

  // For symmetric world
  void convertToImageCoordinates(double real, double &imageMeasure) const
  {
    imageMeasure = std::round(real * scaleX_);
  }

  void convertToRealWorldCoordinates(double imageX, double imageY, double &realX, double &realY) const
  {
    realX = (imageX - offsetX_) / scaleX_;
    realY = (imageY - offsetY_) / scaleY_;
  }

private:
  double realWorldWidth_;
  double realWorldHeight_;
  double imageWidth_;
  double imageHeight_;

  double scaleX_;
  double scaleY_;
  double offsetX_;
  double offsetY_;
};

struct Shelfino
{
  int id;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;
};

struct Gate
{
  double x;
  double y;
};

struct Obstacle
{
  virtual geometry_msgs::msg::Polygon getPolygon()
  {
    geometry_msgs::msg::Polygon polygon;
    return polygon;
  }
};

struct PolygonObstacle : Obstacle
{
  geometry_msgs::msg::Polygon polygon;

  geometry_msgs::msg::Polygon getPolygon()
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

  geometry_msgs::msg::Polygon getPolygon()
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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class EnvironmentMap : public rclcpp::Node
{
public:
  EnvironmentMap()
      : Node("env_map"), coordinateMapper(17, 17, 750, 750)
  {
    obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", 10, std::bind(&EnvironmentMap::getObstacles, this, _1));
    gatesSubscritpion_ = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", 10, std::bind(&EnvironmentMap::getGates, this, _1));
    mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", 10, std::bind(&EnvironmentMap::getMap, this, _1));

    std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0Odom = std::bind(&EnvironmentMap::getShelfinoPosition, this, _1, 0);
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino1Odom = std::bind(&EnvironmentMap::getShelfinoPosition, this, _1, 1);
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino2Odom = std::bind(&EnvironmentMap::getShelfinoPosition, this, _1, 2);

    shelfino0Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0Odom);
    shelfino1Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino1/odom", 10, shelfino1Odom);
    shelfino2Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino2/odom", 10, shelfino2Odom);

    mapUpdateTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EnvironmentMap::updateMap, this));

    cv::namedWindow("Environment Map", cv::WINDOW_AUTOSIZE);
    mapImage = cv::Mat(750, 750, CV_8UC3, cv::Scalar(255, 255, 255));
    mapImageCopy = mapImage.clone();
  }

private:
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0Subscritpion_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino1Subscritpion_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino2Subscritpion_;

  cv::Mat mapImage; // Declare mapImage as a class member
  cv::Mat mapImageCopy;
  std::vector<Shelfino> shelfinos;
  std::vector<Obstacle> obstacles;
  Gate gate;
  geometry_msgs::msg::Polygon mapBorders;

  CoordinateMapper coordinateMapper;
  rclcpp::TimerBase::SharedPtr mapUpdateTimer_;

  // Drawing the map
  void drawPolygon(const geometry_msgs::msg::Polygon &polygon, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
  {
    // Convert polygon points to OpenCV points
    std::vector<cv::Point> cvPoints;
    for (const auto &point : polygon.points)
    {
      double imageX, imageY;
      // fix flipped coordinates
      // TODO: maybe integrate this in the coordinateclass
      coordinateMapper.convertToImageCoordinates(-point.y, -point.x, imageX, imageY);
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
    double imageCenterX, imageCenterY;
    // fix flipped coordinates
    // TODO: maybe integrate this in the coordinateclass
    coordinateMapper.convertToImageCoordinates(-center.y, -center.x, imageCenterX, imageCenterY);

    double imageRadius;
    coordinateMapper.convertToImageCoordinates(radius, imageRadius);

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

      double imageX, imageY;
      coordinateMapper.convertToImageCoordinates(-transformedCorner.y(), -transformedCorner.x(), imageX, imageY);
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

      Obstacle obstacle;

      if (radius == 0.0)
      {
        this->drawPolygon(polygon, mapImage, true, cv::Scalar(0, 255, 0));
        PolygonObstacle obstacle;
        obstacle.polygon = polygon;
      }
      else
      {
        // Circle only contains one polygon
        const auto &point = polygon.points[0];
        this->drawCircle(radius / 2.0, point, mapImage, true, cv::Scalar(0, 255, 0));
        CylinderObstacle obstacle;
        obstacle.centerX = point.x;
        obstacle.centerY = point.y;
        obstacle.radius = radius / 2.0;
      }

      obstacles.push_back(obstacle);
    }
  }

  void getGates(const geometry_msgs::msg::PoseArray &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received gate");

    const auto &pose = msg.poses[0];
    const auto &position = pose.position;

    auto &orientation = pose.orientation;

    this->drawRectangle(1.0, 1.0, position, orientation, mapImage, true, cv::Scalar(0, 0, 255));
    gate.x = position.x;
    gate.y = position.y;
  }

  void getMap(const geometry_msgs::msg::Polygon &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map borders");

    // Draw map borders
    this->drawPolygon(msg, mapImage, false);
    mapBorders = msg;
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