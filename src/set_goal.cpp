#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SetGoal : public rclcpp::Node
{
    public:
        explicit SetGoal("set_goal"), count_(0), distance_(0.0)
        {
            goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>('move_base_simple/goal', 10);
            timer_ = this->create_wall_timer(
                100ms, 10,std::bind(&SetGoal::callback, this));
            current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SetGoal::get_current_pose, this, _1));
            this->declare_parameter("goal_point_file_");
        }
    private:
        size_t count_;
        double distance_;

        auto file_path_;

        this->get_parameter("goal_point_file_", file_path_);
        node_ = YAML::LoadFile(file_path_);
        goal_points_ = node_["points"].as<std::vector<std::vector<float>>>();

        auto goal_stamped_ = geometry_msgs::msg::PoseStamped();

        void callback()
        {
            for(const auto & points_list: goal_points_){
                for(const auto & itr : points_list)
                {
                    goal_stamped_.header.stamp = get_clock()->now();
                    goal_stamped_.header.frame_id = "map";
                    goal_stamped_.pose.position.x = itr->position.x;
                    goal_stamped_.pose.position.y = itr->position.y;
                    goal_stamped_.pose.position.z = itr->position.z;
                    goal_stamped_.pose.orientation.x = itr->orientation.x;
                    goal_stamped_.pose.orientation.y = itr->orientation.y;
                    goal_stamped_.pose.orientation.z = itr->orientation.z;
                    goal_stamped_.pose.orientation.w = itr->orientation.w;

                    goal_publisher_->publish(goal_stamped);

                }
            }

        }

        void get_current_pose(const nav_msgs::msg::Odometry::SharedPtr data) const
        {

            

        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_;
        rclcpp::Clock clock_(RCL_ROS_TIME);
        std::vector<std::vector<float>> goal_points_;

        YAML::Node node_;
        bool change_waypoint_flg_ = false;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetGoal>());
    rclcpp::shutdown();
    return 0;
}