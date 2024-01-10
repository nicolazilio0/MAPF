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
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino0_path = std::bind(&Orchestrator::get_path, this, _1, 0);
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino1_path = std::bind(&Orchestrator::get_path, this, _1, 1);
        std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> shelfino2_path = std::bind(&Orchestrator::get_path, this, _1, 2);

        shelfino0_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino0/plan1", 10, shelfino0_path);
        shelfino1_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino1/plan1", 10, shelfino1_path);
        shelfino2_path_subscritpion = this->create_subscription<nav_msgs::msg::Path>("shelfino2/plan1", 10, shelfino2_path);

        shelfino0_action_client = rclcpp_action::create_client<FollowPath>(this, "shelfino0/follow_path");
        shelfino1_action_client = rclcpp_action::create_client<FollowPath>(this, "shelfino1/follow_path");
        shelfino2_action_client = rclcpp_action::create_client<FollowPath>(this, "shelfino2/follow_path");

        // Wait for the action server to become available
        if (!shelfino0_action_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino1 not available");
            rclcpp::shutdown();
        }
        if (!shelfino1_action_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino2 not available");
            rclcpp::shutdown();
        }
        if (!shelfino2_action_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server shelfino3 not available");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Action server OK");

        shelfino0_path_recived = false;
        shelfino1_path_recived = false;
        shelfino2_path_recived = false;

        orchestrator_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Orchestrator::send_plan, this));
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino0_action_client;
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino1_action_client;
    rclcpp_action::Client<FollowPath>::SharedPtr shelfino2_action_client;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino0_path_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino1_path_subscritpion;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr shelfino2_path_subscritpion;

    bool shelfino0_path_recived;
    nav_msgs::msg::Path shelfino0_path;

    bool shelfino1_path_recived;
    nav_msgs::msg::Path shelfino1_path;

    bool shelfino2_path_recived;
    nav_msgs::msg::Path shelfino2_path;

    rclcpp::TimerBase::SharedPtr orchestrator_timer;
    rclcpp::TimerBase::SharedPtr path0_delay_timer;
    rclcpp::TimerBase::SharedPtr path1_delay_timer;
    rclcpp::TimerBase::SharedPtr path2_delay_timer;

    void get_path(const nav_msgs::msg::Path::SharedPtr &msg, int robotId)
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

    double getDelay(int delay_steps, double step_discr = 0.1, double const_velocity = 0.2)
    {
        return static_cast<double>(delay_steps) * step_discr / const_velocity;
    }

    void checkCollisions(nav_msgs::msg::Path &path0, nav_msgs::msg::Path &path1, nav_msgs::msg::Path &path2, double &path0_delay, double &path1_delay, double &path2_delay, double safety_dis = 1)
    {
        int max_timestep = std::max({static_cast<int>(path0.poses.size()),
                                     static_cast<int>(path1.poses.size()),
                                     static_cast<int>(path1.poses.size())}) *
                           2;

        int collision_delay = 2;

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
                // once the collisions are fixed
                // add to the front of the path the initial pose for the delay timestep calculated

                path0_delay = getDelay(path0_delay);
                path1_delay = getDelay(path1_delay);
                path2_delay = getDelay(path2_delay);

                std::cout << path0_delay << "s , " << path1_delay << "s , " << path2_delay << "s" << std::endl;
            }
        }
    }

    void send_plan()
    {

        if (shelfino0_path_recived && shelfino1_path_recived && shelfino2_path_recived)
        {
            RCLCPP_INFO(this->get_logger(), "Checking and sending paths");
            orchestrator_timer->cancel();

            double path0_delay = 0, path1_delay = 0, path2_delay = 0;

            checkCollisions(shelfino0_path, shelfino1_path, shelfino2_path, path0_delay, path1_delay, path2_delay);

            path0_delay_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(path0_delay * 1000)), [this]()
                                                        { send_goal(0); });
            path1_delay_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(path1_delay * 1000)), [this]()
                                                        { send_goal(1); });
            path2_delay_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(path2_delay * 1000)), [this]()
                                                        { send_goal(2); });
        }
    }

    void send_goal(int robot_id)
    {
        FollowPath::Goal goal_msg;
        goal_msg.controller_id = "FollowPath";

        switch (robot_id)
        {
        case 0:
            path0_delay_timer->cancel();
            goal_msg.path = shelfino0_path;
            shelfino0_action_client->async_send_goal(goal_msg);
            break;
        case 1:
            path1_delay_timer->cancel();
            goal_msg.path = shelfino1_path;
            shelfino1_action_client->async_send_goal(goal_msg);
            break;
        case 2:
            path2_delay_timer->cancel();
            goal_msg.path = shelfino2_path;
            shelfino2_action_client->async_send_goal(goal_msg);
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
