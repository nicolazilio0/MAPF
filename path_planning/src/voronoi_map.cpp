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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

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
      : Node("voronoi_map"), coordinateMapper(16, 16, 500, 500)
  {
    obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", 10, std::bind(&VoronoiMap::getObstacles, this, _1));
    gatesSubscritpion_ = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", 10, std::bind(&VoronoiMap::getGates, this, _1));
    mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", 10, std::bind(&VoronoiMap::getMap, this, _1));

    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    mapImage = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
  }

private:
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscritpion_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;

  cv::Mat mapImage; // Declare mapImage as a class member
  CoordinateMapper coordinateMapper;

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

  void drawCircle(float radius, const cv::Point &center, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
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

  void drawSquare(double width, double height, const cv::Point &center, const geometry_msgs::msg::Quaternion &orientation, cv::Mat &image, bool fill, const cv::Scalar &color = cv::Scalar(0, 0, 0))
  {
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    // Compute the four corners of the rotated square
    double halfWidth = width / 2.0;
    double halfHeight = height / 2.0;

    Eigen::Vector3d p1(-halfWidth, -halfHeight, 0);
    Eigen::Vector3d p2(halfWidth, -halfHeight, 0);
    Eigen::Vector3d p3(halfWidth, halfHeight, 0);
    Eigen::Vector3d p4(-halfWidth, halfHeight, 0);

    p1 = (rotationMatrix * p1).eval();
    p2 = (rotationMatrix * p2).eval();
    p3 = (rotationMatrix * p3).eval();
    p4 = (rotationMatrix * p4).eval();

    double x1 = center.x + p1.x();
    double y1 = center.y + p1.y();

    double x2 = center.x + p2.x();
    double y2 = center.y + p2.y();

    double x3 = center.x + p3.x();
    double y3 = center.y + p3.y();

    double x4 = center.x + p4.x();
    double y4 = center.y + p4.y();

    // Convert square points to OpenCV points
    std::vector<cv::Point> cvPoints;
    double imageX, imageY;
    coordinateMapper.convertToImageCoordinates(-y1, -x1, imageX, imageY);
    cvPoints.emplace_back(imageX, imageY);
    coordinateMapper.convertToImageCoordinates(-y2, -x2, imageX, imageY);
    cvPoints.emplace_back(imageX, imageY);
    coordinateMapper.convertToImageCoordinates(-y3, -x3, imageX, imageY);
    cvPoints.emplace_back(imageX, imageY);
    coordinateMapper.convertToImageCoordinates(-y4, -x4, imageX, imageY);
    cvPoints.emplace_back(imageX, imageY);

    // Draw square on the image
    if (fill)
    {
      cv::fillConvexPoly(image, cvPoints, color);
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
      const auto &obstacle = msg.obstacles[i];

      // Access the polygon and radius fields for each obstacle
      const auto &polygon = obstacle.polygon;
      const auto &radius = obstacle.radius;

      if (radius == 0.0)
      {
        this->drawPolygon(polygon, mapImage, true, cv::Scalar(0, 255, 0));
      }
      else
      {
        // Circle only contains one polygon
        const auto &point = polygon.points[0];
        this->drawCircle(radius / 2, cv::Point(point.x, point.y), mapImage, true, cv::Scalar(0, 255, 0));
      }
    }

    // Plot the map
    cv::imshow("map", mapImage);
    cv::waitKey(0);
  }

  void getGates(const geometry_msgs::msg::PoseArray &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received gate");

    const auto &pose = msg.poses[0];
    const auto &position = pose.position;
    // always 0 0 0 for now
    const auto &orientation = pose.orientation;

    // is a box of size 1 [mt] -> use circle to simplify
    this->drawSquare(1, 1, cv::Point(position.x, position.y), orientation, mapImage, true, cv::Scalar(0, 0, 255));
  }

  void getMap(const geometry_msgs::msg::Polygon &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map borders");

    // Draw map borders
    this->drawPolygon(msg, mapImage, true, cv::Scalar(255, 255, 255));
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto voronoiMap = std::make_shared<VoronoiMap>();
  rclcpp::spin(voronoiMap);
  rclcpp::shutdown();
  return 0;
}