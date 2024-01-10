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
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino0Path = std::bind(&Orchestrator::getPath, this, _1, 0);

        shelfino0PathSubscritpion_ = this->create_subscription<nav_msgs::msg::Path>("shelfino0/plan1", 10, shelfino0Path);

        shelfino0_action_client_ = rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");

        // Wait for the action server to become available
        if (!shelfino0_action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino1 not available");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Action server OK");

        shelfino0_path_recived = false;

        orchestratorTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Orchestrator::sendPlan, this));
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino0_action_client_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0PathSubscritpion_;
    bool shelfinos_path_recived[1];
    bool shelfino0_path_recived;
    nav_msgs::msg::Path shelfino0_path;

    rclcpp::TimerBase::SharedPtr orchestratorTimer_;
    rclcpp::TimerBase::SharedPtr path0_delay_timer_;

    void getPath(const nav_msgs::msg::Path::SharedPtr &msg, int robotId)
    {
        RCLCPP_INFO(this->get_logger(), "Received path map for %i", robotId);

        switch (robotId)
        {
        case 0:
            shelfino0_path_recived = true;
            shelfino0_path = *msg;
            break;
        }
    }

    double getDelay(int delay_steps, double step_discr = 0.1, double const_velocity = 0.2)
    {
        return static_cast<double>(delay_steps) * step_discr / const_velocity;
    }

    void sendPlan()
    {

        if (shelfino0_path_recived)
        {
            RCLCPP_INFO(this->get_logger(), "Checking and sending paths");
            orchestratorTimer_->cancel();

            double path0_delay = 0;

            path0_delay = getDelay(path0_delay);
            // checkCollisions(shelfino0_path, shelfino1_path, shelfino2_path, path0_delay, path1_delay, path2_delay);

            path0_delay_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(path0_delay * 1000)), [this]()
                                                         { sendGoal(0); });
        }
    }

    void sendGoal(int robot_id)
    {
        FollowPath::Goal goal_msg;
        goal_msg.controller_id = "FollowPath";

        switch (robot_id)
        {
        case 0:
            path0_delay_timer_->cancel();
            goal_msg.path = shelfino0_path;
            shelfino0_action_client_->async_send_goal(goal_msg);
            break;
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