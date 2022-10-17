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

class MoveGoal : public rclcpp::Node
{
    public:
        explicit MoveGoal("move_goal"), count_(0), distance_(0.0)
        {
            goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>('move_base_simple/goal', 10);
            current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SetGoal::callback, this, _1));
            this->declare_parameter("goal_point_file_");
            this->get_parameter("goal_point_file_", file_path_);
            this->declare_parameter("goal_tolerance_", 2.0);
            this->get_parameter("goal_tolerance_", goal_tolerance_);
            node_ = YAML::LoadFile(file_path_);
            goal_points_ = node_["points"].as<std::vector<std::vector<double>>>();
        }
    private:
        void callback(const nav_msgs::msg::Odometry::SharedPtr data) const
        {
            point_ = data->pose.pose.position;
            distance_ = get_disctance(goal_points_[count_][0], goal_points_[count_][1], point_);
            if(distance_ > goal_tolerance_){
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
            else{
                count_++;
            }

        }

        double get_disctance(double x, double y, geometry_msgs::msg::Point point)
        {
            return std::hypot((x - point.x), (y - point.y));

        }

        auto goal_stamped_ = geometry_msgs::msg::PoseStamped();
        geometry_msgs::msg::Point point_;

        size_t count_;
        double distance_;

        std::string file_path_;
        double goal_tolerance_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_;
        
        std::vector<std::vector<float>> goal_points_;

        YAML::Node node_;
        bool change_waypoint_flg_ = false;

        double x_;
        double y_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveGoal>());
    rclcpp::shutdown();
    return 0;
}