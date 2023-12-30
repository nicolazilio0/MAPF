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
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using std::placeholders::_1;

class Orchestrator : public rclcpp::Node
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;

    Orchestrator() : Node("path_follower_node")
    {
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino0RRTStar = std::bind(&Orchestrator::getPath, this, _1, 0);
        shelfino0RRTStarSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino0/rrtstar_path", 10, shelfino0RRTStar);

        action_client_ = rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");

        // Wait for the action server to become available
        if (!action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Action server OK");

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp_action::Client<FollowPath>::SharedPtr action_client_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0RRTStarSubscritpion_;

    void getPath(const nav_msgs::msg::Path::SharedPtr &msg, int robotId)
    {
        RCLCPP_INFO(this->get_logger(), "Received path map for %i", robotId);

        path_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Sended path");

        // Create a FollowPath action goal
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = *msg;
        goal_msg.controller_id = "FollowPath";
        action_client_->async_send_goal(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Sended action");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto orchestrator = std::make_shared<Orchestrator>();
    rclcpp::spin(orchestrator);
    rclcpp::shutdown();
    return 0;
}
