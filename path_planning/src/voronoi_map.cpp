#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>

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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class VoronoiMap : public rclcpp::Node
{
public:
  VoronoiMap()
      : Node("voronoi_map"), coordinateMapper(30, 30, 1000, 1000)
  {
    obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", 10, std::bind(&VoronoiMap::getObstacles, this, _1));
    gatesSubscritpion_ = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", 10, std::bind(&VoronoiMap::getGates, this, _1));
    mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", 10, std::bind(&VoronoiMap::getMap, this, _1));

    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    mapImage = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
  }

private:
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;

  cv::Mat mapImage; // Declare mapImage as a class member
  CoordinateMapper coordinateMapper;

  // Drawing the map
  void drawPolygon(const geometry_msgs::msg::Polygon &polygon, cv::Mat &image)
  {
    // Convert polygon points to OpenCV points
    std::vector<cv::Point> cvPoints;
    for (const auto &point : polygon.points)
    {
      double imageX, imageY;
      coordinateMapper.convertToImageCoordinates(point.x, point.y, imageX, imageY);
      cvPoints.emplace_back(imageX, imageY);
    }

    // Draw polygon on the image
    cv::polylines(image, cvPoints, true, cv::Scalar(0, 255, 0), 2);
  }

  void drawCircle(float radius, const cv::Point &center, cv::Mat &image)
  {
    // Draw circle on the image
    double imageCenterX, imageCenterY;
    coordinateMapper.convertToImageCoordinates(center.x, center.y, imageCenterX, imageCenterY);

    double imageRadius;
    coordinateMapper.convertToImageCoordinates(radius, imageRadius);

    cv::circle(image, cv::Point(imageCenterX, imageCenterY), static_cast<int>(imageRadius), cv::Scalar(0, 0, 255), 2);
  }

  // Callbacks for topic handling
  void getObstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
  {

    // Create an OpenCV Mat for visualization
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    RCLCPP_INFO(this->get_logger(), "Received %zu obstacles", msg.obstacles.size());

    for (size_t i = 0; i < msg.obstacles.size(); i++)
    {
      const auto &obstacle = msg.obstacles[i];

      // Access the polygon and radius fields for each obstacle
      const auto &polygon = obstacle.polygon;
      const auto &radius = obstacle.radius;
      // Information log
      // // Log information about the obstacle
      // RCLCPP_INFO(this->get_logger(), "Obstacle %zu:", i);
      // // Log polygon information
      // for (const auto &point : polygon.points)
      // {
      //     RCLCPP_INFO(this->get_logger(), "  Polygon Point: (%f, %f)", point.x, point.y);
      // }
      // // Log radius information
      // RCLCPP_INFO(this->get_logger(), "  Radius: %f", radius);
      if (radius == 0.0)
      {
        // drawPolygon(polygon, image);
        // radius = 0;
        RCLCPP_INFO(this->get_logger(), "  Polygon ");
      }
      else
      {
        // Circle only contains one polygon
        const auto &point = polygon.points[0];
        RCLCPP_INFO(this->get_logger(), "  Circle center: (%f, %f), Radius (%f)", point.x, point.y, radius);

        drawCircle(radius/2, cv::Point(point.x, point.y), mapImage);
      }
    }
    cv::imshow("map", mapImage); // Declare mapImage as a class member);
    cv::waitKey(0);
  }

  void getGates(const geometry_msgs::msg::PoseArray &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I recived gates");
  }

  void getMap(const geometry_msgs::msg::Polygon &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map borders");

    const auto &polygon = msg;

    for (const auto &point : polygon.points)
    {
      RCLCPP_INFO(this->get_logger(), "  Polygon Point: (%f, %f)", point.x, point.y);
    }

    // Draw map borders
    drawPolygon(polygon, mapImage);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiMap>());
  rclcpp::shutdown();
  return 0;
}