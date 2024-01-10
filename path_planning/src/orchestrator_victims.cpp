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
#include <vector>
#include <algorithm>
#include <limits>

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
        shelfino0_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino0/plan1", 10, std::bind(&Orchestrator::get_path, this, _1));

        shelfino0_action_client = rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");

        // Wait for the action server to become available
        if (!shelfino0_action_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino1 not available");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Action server OK");

        shelfino0_path_recived = false;

        orchestrator_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Orchestrator::send_plan, this));
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino0_action_client;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0_path_subscritpion;

    bool shelfino0_path_recived;
    nav_msgs::msg::Path shelfino0_path;

    rclcpp::TimerBase::SharedPtr orchestrator_timer;
    rclcpp::TimerBase::SharedPtr path0_delay_timer;

    void get_path(const nav_msgs::msg::Path &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received path map for 0");

        shelfino0_path_recived = true;
        shelfino0_path = msg;
    }

    void send_plan()
    {

        if (shelfino0_path_recived)
        {
            RCLCPP_INFO(this->get_logger(), "Checking and sending paths");
            orchestrator_timer->cancel();

            FollowPath::Goal goal_msg;
            goal_msg.controller_id = "FollowPath";
            goal_msg.path = shelfino0_path;

            shelfino0_action_client->async_send_goal(goal_msg);
        }
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