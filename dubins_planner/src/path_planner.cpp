#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "dubins.cpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/action/follow_path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"



class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("path_talker")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "shelfinoG/transform", qos, std::bind(&PathPublisher::handle_transform, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("shelfinoG/plan", 10);
    }

  
  private:

    void handle_transform(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
    {
        t = *msg;
        
        if(!data_sent){
            data_sent = true;

            std::cout << "x: " << t.transform.translation.x << std::endl;
            std::cout << "y: " << t.transform.translation.y << std::endl;

            tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            Dubins_curve curve;
            curve = dubins_shortest_path(t.transform.translation.x, t.transform.translation.y, yaw, 0, 0, M_PI+M_PI/4 ,3);
            nav_msgs::msg::Path path_msg = plot_dubins(curve);

            std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
            path_msg.header.stamp = this->get_clock()->now();
            path_msg.header.frame_id = "map";

            using FollowPath = nav2_msgs::action::FollowPath;
            // using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

            rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;

            client_ptr_ = rclcpp_action::create_client<FollowPath>(this,"shelfinoG/follow_path");

            if (!client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            publisher_->publish(path_msg);
            sleep(0.5);
            publisher_->publish(path_msg);

            auto goal_msg = FollowPath::Goal();
            goal_msg.path = path_msg;
            goal_msg.controller_id = "FollowPath";

            RCLCPP_INFO(this->get_logger(), "Sending goal");

            client_ptr_->async_send_goal(goal_msg);
            sleep(0.5);
            client_ptr_->async_send_goal(goal_msg);
            
        }

        return;
    }

    Plot_arc_struct plot_arc(Dubins_arc arc){
    
        float pts[npts+1][2] = {};
        for(int j = 0; j <= npts; j++){
            float s = arc.L/npts * j;
            Circle_line ci;
            ci = circline(s, arc.x0, arc.y0, arc.th0, arc.k);
            float x = ci.x;
            float y = ci.y;
            pts[j][0] = x;
            pts[j][1] = y;
        }
        
        Plot_arc_struct a;
        for(int r= 0; r<npts; r++){
            for(int c = 0; c<2; c++){
                a.pts[r][c] = pts[r][c];
            }
            
        }  
        return a;
    }

    nav_msgs::msg::Path plot_dubins(Dubins_curve curve){
      float ret[((npts+1)*3)][2];
      Plot_arc_struct a1 = plot_arc(curve.a1);
      for(int r= 0; r<npts; r++){
          for(int c = 0; c<2; c++){
              cout<<a1.pts[r][c]<<"\t";
              
              ret[r][c] = a1.pts[r][c];
              
          }
          cout<<"\n\n";
          
      }

      Plot_arc_struct a2 = plot_arc(curve.a2);
      for(int r= 0; r<npts; r++){
          for(int c = 0; c<2; c++){
              cout<<a2.pts[r][c]<<"\t";
              ret[r+100][c] = a2.pts[r][c];
              
          }
          cout<<"\n\n";
      }
      Plot_arc_struct a3 = plot_arc(curve.a3);
      for(int r= 0; r<npts; r++){
          for(int c = 0; c<2; c++){
              cout<<a3.pts[r][c]<<"\t";
              ret[r+200][c] = a3.pts[r][c];
          }
          cout<<"\n\n";
      }
      cout<<"******************";
      for(int r= 0; r<(npts)*3; r++){
          for(int c = 0; c<2; c++){
              cout<<ret[r][c]<<"\t";
          }
          cout<<"\n";
      }

      nav_msgs::msg::Path path_msg;
      std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
      path_msg.header.stamp = this->get_clock()->now();
      path_msg.header.frame_id = "map";
      geometry_msgs::msg::Pose pose_temp;
      geometry_msgs::msg::Point position_temp;
      geometry_msgs::msg::Quaternion quaternion_temp;
      geometry_msgs::msg::PoseStamped pose_stamped_temp;

      for(int i = 0; i<((npts)*3); i++) {
            
            position_temp.x = ret[i][0];
            
            position_temp.y = ret[i][1];
            position_temp.z = 0 ;

            quaternion_temp.x =  0.0;
            quaternion_temp.y =  0.0;
            quaternion_temp.z =  0.0;
            quaternion_temp.w =  0.0;

            if(i == ((npts)*3)-1){
                quaternion_temp.z =  0.9252115;
                quaternion_temp.w = -0.3794518;
            }

            pose_temp.position = position_temp;
            pose_temp.orientation = quaternion_temp;

            pose_stamped_temp.pose = pose_temp;
            pose_stamped_temp.header.stamp = this->get_clock()->now();
            pose_stamped_temp.header.frame_id = "";

            if (i <((npts)*3)-30 || i == ((npts)*3)-1){
                poses_temp.push_back(pose_stamped_temp);
            }

        }
        path_msg.poses = poses_temp;

        return path_msg;
    }
    
    bool data_sent = false;
    geometry_msgs::msg::TransformStamped t;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}