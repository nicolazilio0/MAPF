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

        shelfino0_path_recived = false;
        shelfino1_path_recived = false;
        shelfino2_path_recived = false;

        orchestratorTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Orchestrator::sendPlan, this));
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino0_action_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino1_action_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino2_action_client_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0PathSubscritpion_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino1PathSubscritpion_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino2PathSubscritpion_;
    bool shelfinos_path_recived[3];
    bool shelfino0_path_recived;
    nav_msgs::msg::Path shelfino0_path;

    bool shelfino1_path_recived;
    nav_msgs::msg::Path shelfino1_path;

    bool shelfino2_path_recived;
    nav_msgs::msg::Path shelfino2_path;

    rclcpp::TimerBase::SharedPtr orchestratorTimer_;

    void getPath(const nav_msgs::msg::Path::SharedPtr &msg, int robotId)
    {
        RCLCPP_INFO(this->get_logger(), "Received path map for %i", robotId);

        switch (robotId)
        {
        case 0:
            shelfino0_path_recived = true;
            shelfino0_path = *msg;
            break;
        case 1:
            shelfino1_path_recived = true;
            shelfino1_path = *msg;
            break;
        case 2:
            shelfino2_path_recived = true;
            shelfino2_path = *msg;
            break;
        }
    }

    void checkCollisions(nav_msgs::msg::Path &path0, nav_msgs::msg::Path &path1, nav_msgs::msg::Path &path2, double safety_dis = 0.5)
    {
        int max_timestep = std::max({static_cast<int>(path0.poses.size()),
                                     static_cast<int>(path1.poses.size()),
                                     static_cast<int>(path1.poses.size())}) *
                           2;

        int collision_delay = 2;
        int path0_delay = 100, path1_delay = 0, path2_delay = 0;

        bool check_collision = true;

        while (check_collision)
        {
            std::cout << "checking..." << std::endl;

            nav_msgs::msg::Path path0_eq, path1_eq, path2_eq;

            path0_eq.header = shelfino0_path.header;
            path1_eq.header = shelfino1_path.header;
            path2_eq.header = shelfino2_path.header;

            // Fill the delay timestep with initial position (delay = 0 at the begin)
            path0_eq.poses.insert(path0_eq.poses.end(), path0_delay, path0.poses.front());
            path1_eq.poses.insert(path1_eq.poses.end(), path1_delay, path1.poses.front());
            path2_eq.poses.insert(path2_eq.poses.end(), path2_delay, path2.poses.front());

            // From the delay timestep, copy the original path
            path0_eq.poses.insert(path0_eq.poses.end(), path0.poses.begin(), path0.poses.end());
            path1_eq.poses.insert(path1_eq.poses.end(), path1.poses.begin(), path1.poses.end());
            path2_eq.poses.insert(path2_eq.poses.end(), path2.poses.begin(), path2.poses.end());

            std::vector<std::vector<double>> dist(max_timestep, std::vector<double>(3));

            // for each timestep calculate the path distance
            for (int t = 0; t < max_timestep; ++t)
            {
                dist[t][0] = std::hypot(path0_eq.poses[t].pose.position.x - path1_eq.poses[t].pose.position.x,
                                        path0_eq.poses[t].pose.position.y - path1_eq.poses[t].pose.position.y);

                dist[t][1] = std::hypot(path0_eq.poses[t].pose.position.x - path2_eq.poses[t].pose.position.x,
                                        path0_eq.poses[t].pose.position.y - path2_eq.poses[t].pose.position.y);

                dist[t][2] = std::hypot(path1_eq.poses[t].pose.position.x - path2_eq.poses[t].pose.position.x,
                                        path1_eq.poses[t].pose.position.y - path2_eq.poses[t].pose.position.y);
            }

            int collision_case = -1;

            // get the timestep of collision
            for (int t = 0; t < max_timestep; ++t)
            {
                // check if the distance is 0.001 < x < 0.353 (0.001 since when we calculate distance between point and null we get 4.6e-310 but is not a collision)
                // plus, we want to anticipate the collision so when is near 0.353 we consider it as an imminent collision
                if (dist[t][0] < safety_dis && dist[t][0] > 0.001 && dist[t][1] < safety_dis && dist[t][1] > 0.001 && dist[t][2] < safety_dis && dist[t][2] > 0.001)
                {
                    collision_case = t;
                    break;
                    // Get only the first collision, we will check if the added delay fix the others
                }
            }

            // check the collision case
            if (collision_case != -1)
            {
                std::cout << "fixing collisions " << dist[collision_case][0] << std::endl;

                if (dist[collision_case][0] < safety_dis) // collision btwn path0 and path1
                {
                    if (path0.poses.size() < path1.poses.size())
                    {
                        path0_delay += collision_delay;
                    }
                    else
                    {
                        path1_delay += collision_delay;
                    }
                }
                else if (dist[collision_case][1] < safety_dis) // collision btwn path0 and path2
                {
                    if (path0.poses.size() < path2.poses.size())
                    {
                        path0_delay += collision_delay;
                    }
                    else
                    {
                        path2_delay += collision_delay;
                    }
                }
                else if (dist[collision_case][2] < safety_dis) // collision btwn path1 and path2
                {
                    if (path1.poses.size() < path2.poses.size())
                    {
                        path1_delay += collision_delay;
                    }
                    else
                    {
                        path2_delay += collision_delay;
                    }
                }
            }
            else
            {
                check_collision = false;
                std::cout << "NO collisions" << std::endl;
                std::cout << path0_delay << " , " << path1_delay << " , " << path2_delay << std::endl;
                // once the collisions are fixed
                // add to the front of the path the initial pose for the delay timestep calculated

                path0 = path0_eq;
                path1 = path1_eq;
                path2 = path2_eq;
            }
        }
    }

    void sendPlan()
    {

        if (shelfino0_path_recived && shelfino1_path_recived && shelfino2_path_recived)
        {
            RCLCPP_INFO(this->get_logger(), "Checking and sending paths");
            orchestratorTimer_->cancel();

            RCLCPP_INFO(this->get_logger(), "%i", static_cast<int>(shelfino0_path.poses.size()));

            checkCollisions(shelfino0_path, shelfino1_path, shelfino2_path);

            RCLCPP_INFO(this->get_logger(), "%i", static_cast<int>(shelfino0_path.poses.size()));

            for (int i = 0; i < 10; ++i)
            {
                const auto &pose = shelfino0_path.poses[i];
                std::cout << "Pose " << i + 1 << ":\n";
                std::cout << "  Position (x, y, z): " << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << "\n";
                std::cout << "  Orientation (x, y, z, w): " << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", "
                          << pose.pose.orientation.z << ", " << pose.pose.orientation.w << "\n";
            }

            auto goal_msg = FollowPath::Goal();
            goal_msg.controller_id = "FollowPath";

            goal_msg.path = shelfino0_path;
            shelfino0_action_client_->async_send_goal(goal_msg);

            goal_msg.path = shelfino1_path;
            shelfino1_action_client_->async_send_goal(goal_msg);

            goal_msg.path = shelfino2_path;
            shelfino2_action_client_->async_send_goal(goal_msg);
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
