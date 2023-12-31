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
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino0Path = std::bind(&Orchestrator::getPath, this, _1, 0);
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino1Path = std::bind(&Orchestrator::getPath, this, _1, 1);
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino2Path = std::bind(&Orchestrator::getPath, this, _1, 2);

        shelfino0PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino0/plan1", 10, shelfino0Path);
        shelfino1PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino1/plan1", 10, shelfino1Path);
        shelfino2PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino2/plan1", 10, shelfino2Path);

        shelfino0_action_client_ = rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");
        shelfino1_action_client_ = rclcpp_action::create_client<FollowPath>(this, "shelfino1/follow_path");
        shelfino2_action_client_ = rclcpp_action::create_client<FollowPath>(this, "shelfino2/follow_path");

        // Wait for the action server to become available
        if (!shelfino0_action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino1 not available");
            rclcpp::shutdown();
        }
        if (!shelfino1_action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino2 not available");
            rclcpp::shutdown();
        }
        if (!shelfino2_action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino3 not available");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Action server OK");
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino0_action_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino1_action_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino2_action_client_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0PathSubscritpion_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino1PathSubscritpion_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino2PathSubscritpion_;

    void getPath(const nav_msgs::msg::Path::SharedPtr &msg, int robotId)
    {
        RCLCPP_INFO(this->get_logger(), "Received path map for %i", robotId);

        // Create a FollowPath action goal
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = *msg;
        goal_msg.controller_id = "FollowPath";

        switch (robotId)
        {
        case 0:
            shelfino0_action_client_->async_send_goal(goal_msg);
            break;
        case 1:
            shelfino1_action_client_->async_send_goal(goal_msg);
            break;
        case 2:
            shelfino2_action_client_->async_send_goal(goal_msg);
            break;
        }
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
