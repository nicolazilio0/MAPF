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

#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

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

#include "CoordinateMapper.hpp"
#include "DubinsPath.hpp"
#include "RRTStarDubins.hpp"

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

struct Obstacle
{
    virtual std::vector<double> getObstacle() const = 0;
};

struct PolygonObstacle : Obstacle
{
    geometry_msgs::msg::Polygon polygon;

    std::vector<double> getObstacle() const override
    {

        // Initialize the bounding box coordinates
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        // Calculate the bounding box
        for (const auto &point : polygon.points)
        {
            minX = std::min(minX, static_cast<double>(point.x));
            maxX = std::max(maxX, static_cast<double>(point.x));
            minY = std::min(minY, static_cast<double>(point.y));
            maxY = std::max(maxY, static_cast<double>(point.y));
        }

        // Calculate the center of the bounding box
        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        // Calculate the radius (diameter/2) of the bounding circle
        double radius = std::sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY)) / 2.0;

        // Return the vector containing center_x, center_y, and radius
        return {centerX, centerY, radius};
    }
};

struct CylinderObstacle : Obstacle
{
    double centerX;
    double centerY;
    double radius;

    std::vector<double> getObstacle() const override
    {
        return {centerX, centerY, radius};
    }
};


struct Victim
{
    double centerX;
    double centerY;
    double radius=0.5;
    double value;


};


struct MapBorder
{
    geometry_msgs::msg::Polygon polygon;

    std::vector<std::vector<double>> discretizeBorder(int discretizationPoint = 20, double radius = 0.01)
    {
        std::vector<std::vector<double>> points;

        for (size_t i = 0; i < polygon.points.size(); ++i)
        {
            const auto &startPoint = polygon.points[i];
            const auto &endPoint = polygon.points[(i + 1) % polygon.points.size()];

            // Calculate intermediate points along the edge for (int j = 0; j < numPointsPerEdge; ++j)
            for (int j = 0; j < discretizationPoint; ++j)
            {
                double x = startPoint.x + (endPoint.x - startPoint.x) * static_cast<double>(j) / discretizationPoint;
                double y = startPoint.y + (endPoint.y - startPoint.y) * static_cast<double>(j) / discretizationPoint;

                points.push_back({x, y, radius});
            }
        }

        return points;
    }
};

class RRTStarDubinsPlanner : public rclcpp::Node
{
public:
    RRTStarDubinsPlanner()
        : Node("rrtstardubins_planner"), coordinateMapper(20, 20, 750, 750)
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        obstaclesSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos, std::bind(&RRTStarDubinsPlanner::getObstacles, this, _1));
        gatesSubscritpion_ = this->create_subscription<geometry_msgs::msg::PoseArray>("gate_position", qos, std::bind(&RRTStarDubinsPlanner::getGates, this, _1));
        mapSubscritpion_ = this->create_subscription<geometry_msgs::msg::Polygon>("map_borders", qos, std::bind(&RRTStarDubinsPlanner::getMap, this, _1));
        victimSubscritpion_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("victims", qos, std::bind(&RRTStarDubinsPlanner::getVictims, this, _1));



        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> shelfino0Odom = std::bind(&RRTStarDubinsPlanner::getShelfinoPosition, this, _1, 0);

        shelfino0PathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan1", 10);

        shelfino0Subscritpion_ = this->create_subscription<nav_msgs::msg::Odometry>("shelfino0/odom", 10, shelfino0Odom);

        obstaclesAquired = false;
        shelfinosAquired = false;
        mapAquired = false;
        gateAquired = false;
        victimsAquired=false;
        rndMin = -10;
        rndMax = 10;

        plannerTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RRTStarDubinsPlanner::generateRoadmap, this));
    }

private:
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscritpion_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscritpion_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscritpion_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victimSubscritpion_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0Subscritpion_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino0PathPublisher_;


    MapBorder mapBorder;
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    std::vector<std::unique_ptr<Victim>> victims;
    std::vector<std::unique_ptr<Victim>> victims_copy;

    std::vector<Shelfino> shelfinos;
    Gate gate;

    CoordinateMapper coordinateMapper;
    DubinsPath dubinsPath;
    rclcpp::TimerBase::SharedPtr plannerTimer_;

    bool mapAquired;
    bool obstaclesAquired;
    bool gateAquired;
    bool shelfinosAquired;
    bool victimsAquired;
    double rndMin;
    double rndMax;

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
                auto obstacle = std::make_unique<PolygonObstacle>();
                obstacle->polygon = polygon;
                obstacles.push_back(std::move(obstacle));
            }
            else
            {
                // Circle only contains one polygon
                const auto &point = polygon.points[0];
                auto obstacle = std::make_unique<CylinderObstacle>();
                obstacle->centerX = point.x;
                obstacle->centerY = point.y;
                obstacle->radius = radius;
                obstacles.push_back(std::move(obstacle));
            }
        }

        obstaclesAquired = true;
    }
    void getGates(const geometry_msgs::msg::PoseArray &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received gate");

        const auto &pose = msg.poses[0];
        gate.position = pose.position;
        gate.orientation = pose.orientation;

        gateAquired = true;
    }

    void getMap(const geometry_msgs::msg::Polygon &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map borders");
        mapBorder.polygon = msg;

        mapAquired = true;
    }

    void getShelfinoPosition(const nav_msgs::msg::Odometry::SharedPtr &msg, int robotId)
    {
        auto it = std::find_if(shelfinos.begin(), shelfinos.end(),
                               [robotId](const Shelfino &shelfino)
                               { return shelfino.id == robotId; });

        if (it == shelfinos.end())
        {
            RCLCPP_INFO(this->get_logger(), "Received Shelfino %d odometry", robotId);

            Shelfino shelfino;
            shelfino.id = robotId;
            shelfinos.push_back(shelfino);
            it = std::prev(shelfinos.end());
        }

        it->position = msg->pose.pose.position;
        it->orientation = msg->pose.pose.orientation;

        // Check if messages have been received for all three robots
        if (shelfinos.size() == 1)
        {
            // Unsubscribe from the topics
            shelfino0Subscritpion_.reset();


            RCLCPP_INFO(this->get_logger(), "Recived all Shelfinos");

            shelfinosAquired = true;
        }
    }

    // Callbacks for topic handling
    void getVictims(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received %zu victims", msg.obstacles.size());
        for (size_t i = 0; i < msg.obstacles.size(); i++)
        {
            const auto &msgVictims = msg.obstacles[i];

            // Access the polygon and radius fields for each obstacle
            const auto &polygon = msgVictims.polygon;
            const auto &value = msgVictims.radius;

            // Circle only contains one polygon
            const auto &point = polygon.points[0];
            auto victim = std::make_unique<Victim>();
            victim->centerX = point.x;
            victim->centerY = point.y;
            victim->value = value;
            victims.push_back(std::move(victim));
            victims_copy.push_back(std::move(victim));
        }

        victimsAquired = true;
    }

    // recursive function to try to insert other victims in the path
    std::vector<std::vector<double>> calculate_path_recursive(rrtstar::Node *vicrim_pred, rrtstar::Node *goal, std::vector<std::vector<double>> circular_obstacles ,double residual_time, double value, std::vector<std::vector<double>> path,double rndMax, double rndMin)
    {
        std::vector<std::vector<double>> path_to_victim;
        std::vector<std::vector<double>> best_path=path;

            // loops on all the victims
            for (const auto &victim : victims){
                std::vector<std::vector<double>> tmp_path=path;

                RCLCPP_INFO(this->get_logger(), "Calculating RRT* for victim %f", victim->value);
                std::vector<std::unique_ptr<Victim>>::iterator it = std::find(victims_copy.begin(), victims_copy.end(), victim);
                // while victims copy is not empty and the victim is not in the copy (it means that the victim has already been inserted in the path)
                if(!victims_copy.empty())
                { 
                rrtstar::Node *victim_next = new rrtstar::Node(victim->centerX, victim->centerY,  0.785);
                rrtstar::RRTStarDubins rrtStarDubins(vicrim_pred, victim_next, circular_obstacles, rndMin, rndMax);

                // calculate path from victim to goal
                int counter = 0;

                do
                {
                    path_to_victim = rrtStarDubins.planning(false);
                    counter=counter+1;
                    if(path_to_victim.empty() && counter>=1){
                        break;
                    }
                } while (path_to_victim.empty());

                if(path_to_victim.empty()){
                       continue;
                        

                }
                // calculates the base path to the goal
                std::vector<std::vector<double>> path_victim_to_goal= calculate_path_from_victims_to_goal(goal,victim_next,circular_obstacles,rndMax,rndMin);

                auto path_duration= path_to_victim.size()*0.1/0.2;
                auto path_victim_to_goal_duration= path_victim_to_goal.size()*0.1/0.2;
                residual_time = residual_time - path_duration - path_victim_to_goal_duration;
                
                // for debug
                // RCLCPP_ERROR(this->get_logger(), "Residual time %f", residual_time);
                // if we don't have time then the victim cannot be inserted
                if(residual_time < 0){
                    // RCLCPP_ERROR(this->get_logger(), "Victim %f cannot be reached in time", victim->value);
                    continue;
                }
                else{
                    
                    //for debug
                    // RCLCPP_INFO(this->get_logger(), "Victim %f can be reached in time", victim->value);
                    // RCLCPP_ERROR(this->get_logger(), "Time remaining %f ", residual_time);
                    double value_now = victim->value + value;
                    value=std::max(value_now,value);

                    if(value==value_now)
                    {   
                        RCLCPP_INFO(this->get_logger(), "BEST VALUE");
                        // pushes back the path to save the best path
                        std::reverse(path_to_victim.begin(), path_to_victim.end());
                        tmp_path.insert(tmp_path.end(), path_to_victim.begin(), path_to_victim.end());
                        tmp_path.insert(tmp_path.end(), path_victim_to_goal.begin(), path_victim_to_goal.end());
                        std::reverse(tmp_path.begin(), tmp_path.end());
                        best_path=tmp_path;
                        continue;
                    }
                    else{
                        // we did not have an improvement in the value
                        RCLCPP_INFO(this->get_logger(), "NOT BEST VALUE");
                        continue;

                        }


                    }
                //removes the victim we are taking into consideration from the copy
                std::vector<std::unique_ptr<Victim>>::iterator iter = std::find(victims_copy.begin(), victims_copy.end(), victim);
                if(iter != victims_copy.end()){
                    victims_copy.erase(iter);
                    // RCLCPP_INFO(this->get_logger(), "Trying recursive insertion, victims_size %zu", victims_copy.size());
                    continue;
                    }
                }
            else
            {       
                    // inserts back the victim we are removing to return to a previous state
                    RCLCPP_INFO(this->get_logger(), "ELSE IN THE END");
                    std::vector<std::unique_ptr<Victim>>::iterator asd = std::find(victims.begin(), victims.end(), victim);
                    if(asd != victims.end()){
                    victims_copy.push_back(std::move(*asd));
                    }

                    continue;

                    }
            // pushes back the victim we are taking into consideration from the copy
            std::vector<std::unique_ptr<Victim>>::iterator ti = std::find(victims.begin(), victims.end(), victim);
            if(ti != victims.end()){
                    victims_copy.push_back(std::move(*ti));
                    }
            }

        return best_path;


    }



    // base case of path calculation
    // calculate path from start to victim and from victim to goal
    // calls recursive function to try to insert other victims in the path
    std::vector<std::vector<double>> calculate_path_base(rrtstar::Node *start, rrtstar::Node *goal, std::vector<std::vector<double>> circular_obstacles ,double residual_time,double rndMax, double rndMin)
    {
        std::vector<std::vector<double>> path_to_goal;
        std::vector<std::vector<double>> actual_path;
        double max_value = 0;
        std::vector<std::vector<double>> best_path;
        double remaining_time = residual_time;
        if (remaining_time < 0)
        {   return best_path;
        }
        else
        {
            for (const auto &victim : victims)
            {
                // calculate path from start to victim
                rrtstar::Node *victim_goal = new rrtstar::Node(victim->centerX, victim->centerY,  0.785);
                rrtstar::Node *victim_exit = new rrtstar::Node(victim->centerX, victim->centerY,  0.785);
                rrtstar::RRTStarDubins rrtStarDubins(start, victim_goal, circular_obstacles, rndMin, rndMax);
                int count = 0;
                do
                {
                    path_to_goal = rrtStarDubins.planning(false);
                    count=count+1;
                    if(path_to_goal.empty() && count>=1){
                        break;
                    }
                } while (path_to_goal.empty() || count<1);

                if(path_to_goal.empty()){
                    continue;
                }
                
                // calculates the base path to the goal
                std::vector<std::vector<double>> path_victim_to_goal= calculate_path_from_victims_to_goal(goal,victim_exit,circular_obstacles,rndMax,rndMin);

                std::vector<std::unique_ptr<Victim>>::iterator it = std::find(victims_copy.begin(), victims_copy.end(), victim);
                if(it != victims_copy.end())
                    victims_copy.erase(it);
   
                // RCLCPP_INFO(this->get_logger(), "Trying recursive insertion, victims_size %zu", victims_copy.size());


                auto path_duration= path_to_goal.size()*0.1/0.2;
                auto path_victim_to_goal_duration= path_victim_to_goal.size()*0.1/0.2;
                remaining_time = remaining_time - path_duration - path_victim_to_goal_duration;
                std::reverse(path_to_goal.begin(), path_to_goal.end());
                //if we still have time left we try to insert other victims
                if (remaining_time > 0)
                {
                    double value = victim->value;
                    
                    // keeps track of best victims value
                    max_value=std::max(max_value,value);
                    
                    // recursive call to try to insert other victims
                    auto path_to_goal_recursive = calculate_path_recursive(victim_goal,goal,circular_obstacles,remaining_time,value,path_to_goal,rndMax,rndMin);
                    // keeps track of better value
                    if(max_value==value)
                    {   
                        RCLCPP_INFO(this->get_logger(), "EXIT RECURSION IF");
                        best_path=path_to_goal_recursive;
                        return best_path;

                    }
                    else{
                        RCLCPP_INFO(this->get_logger(), "EXIT RECURSION ELSE");
                    }

                    // for debug
                    // RCLCPP_INFO(this->get_logger(), "Victim %f can be reached in time", victim->value);
                    // RCLCPP_INFO(this->get_logger(), "Time remaining %f ", remaining_time);

                }
                else{
                    // for debug
                    // RCLCPP_INFO(this->get_logger(), "Victim %f cannot be reached in time", victim->value);
                    continue;
                }

            }
            if(best_path.empty()){
                // if we cannot insert other victims we calculate the path from start to goal
                std::vector<std::vector<double>> path_start_to_goal= calculate_path_from_victims_to_goal(goal,start,circular_obstacles,rndMax,rndMin);
                return path_start_to_goal;
            }
            else{
                return best_path;
            }
        }
    }





    // Function for path computing from victims to goal
    std::vector<std::vector<double>> calculate_path_from_victims_to_goal(rrtstar::Node *goal, rrtstar::Node *victim_node ,std::vector<std::vector<double>> circular_obstacles,double rndMax, double rndMin)
    {

        std::vector<std::vector<double>> path_to_goal;
        // computes tha path from the designated victim to the goal position
        rrtstar::RRTStarDubins rrtStarDubins(victim_node, goal, circular_obstacles, rndMin, rndMax);
        do
        {
            path_to_goal = rrtStarDubins.planning(false);
        } while (path_to_goal.empty());
        std::reverse(path_to_goal.begin(), path_to_goal.end());
        return path_to_goal;

    }







    void generateRoadmap()
    {
        if (obstaclesAquired && gateAquired && shelfinosAquired && mapAquired && victimsAquired)
        {
            RCLCPP_INFO(this->get_logger(), "Executing RRT*");
            double max_time = 1000.0; // [s]
            // Stop the timer callback
            plannerTimer_->cancel();

            // Add obstacles to the obstacle list
            std::vector<std::vector<double>> circular_obstacles;

            for (const auto &obstaclePtr : obstacles)
            {
                std::vector<double> obstacle = obstaclePtr->getObstacle();
                circular_obstacles.push_back(obstacle);
            }

            // Discretize map borders and add them to obstacle list
            std::vector<std::vector<double>> discretize_map_borders;

            for (std::vector<double> obstacle : mapBorder.discretizeBorder())
            {
                circular_obstacles.push_back(obstacle);
            }

            auto position = gate.position;
            auto orientation = gate.orientation;

            // Get shelfino's yaw angle
            double gate_t0 = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
            double gate_t1 = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
            double gate_yaw = std::atan2(gate_t0, gate_t1);
            // gate_yaw *= std::signbit(gate.position.y) ? -1 : 1;

            std::cout << "Gate orientation: " << gate_yaw << std::endl;

            rrtstar::Node *goal = new rrtstar::Node(position.x, position.y, gate_yaw);


            auto shelfino = shelfinos[0];

            auto id = shelfino.id;
            auto shelfino_position = shelfino.position;
            auto shelfino_orientation = shelfino.orientation;

            // Get shelfino's yaw angle
            double t0 = 2.0 * (shelfino_orientation.w * shelfino_orientation.z + shelfino_orientation.x * shelfino_orientation.y);
            double t1 = 1.0 - 2.0 * (shelfino_orientation.y * shelfino_orientation.y + shelfino_orientation.z * shelfino_orientation.z);
            double yaw = std::atan2(t0, t1);

            rrtstar::Node *start = new rrtstar::Node(shelfino_position.x, shelfino_position.y, yaw);

            auto start_time = std::chrono::high_resolution_clock::now();

            // Calculate first path from start position to victim for each victim
            std::vector<std::vector<double>> path = calculate_path_base(start, goal , circular_obstacles ,max_time, rndMax , rndMin);

            RCLCPP_INFO(this->get_logger(), "Path size: %zu", path.size());

            auto stop_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);

            std::cout << "Path for shelfino" << id << " founded in: " << duration.count() << "milliseconds" << std::endl;






            // Iterate in reverse the path
            nav_msgs::msg::Path shelfino_path;
            shelfino_path.header.frame_id = "map";
            for (auto it = path.rbegin(); it != path.rend(); ++it)
            {
                const auto &point = *it;

                geometry_msgs::msg::PoseStamped poseStamped;
                // Get x,y
                poseStamped.pose.position.x = point[0];
                poseStamped.pose.position.y = point[1];

                Eigen::Quaterniond quat;
                quat = Eigen::AngleAxisd(point[2], Eigen::Vector3d::UnitZ());
                // Get quaternion orientation from yaw
                poseStamped.pose.orientation.x = quat.x();
                poseStamped.pose.orientation.y = quat.y();
                poseStamped.pose.orientation.z = quat.z();
                poseStamped.pose.orientation.w = quat.w();

                shelfino_path.poses.push_back(poseStamped);
            }

            // Sent the path accordingly to the shelfino's id
            switch (id)
            {
            case 0:
                shelfino0PathPublisher_->publish(shelfino_path);
                break;



        }
        }
    }
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rrtStarDubinsPlanner = std::make_shared<RRTStarDubinsPlanner>();
    rclcpp::spin(rrtStarDubinsPlanner);
    rclcpp::shutdown();
    return 0;
}