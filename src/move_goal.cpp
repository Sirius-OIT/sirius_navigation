#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MoveGoal : public rclcpp::Node
{
    public:
        explicit MoveGoal() : Node("move_goal"), count_(0), distance_(0.0)
        {
            goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
            current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MoveGoal::callback, this, std::placeholders::_1));
            this->declare_parameter("goal_point_file_");
            this->get_parameter("goal_point_file_", file_path_);
            this->declare_parameter("goal_tolerance_", 2.0);
            this->get_parameter("goal_tolerance_", goal_tolerance_);
            node_ = YAML::LoadFile(file_path_);
            goal_points_ = node_["points"].as<std::vector<std::vector<double>>>();
        
        }

        struct Vector2D
        {
            double x_;
            double y_;
        };

    private:
        void callback(const nav_msgs::msg::Odometry::SharedPtr data)
        {
            auto goal_stamped_ = geometry_msgs::msg::PoseStamped();
            vector2d.x_ = data->pose.pose.position.x;
            vector2d.y_ = data->pose.pose.position.y;
            distance_ = get_disctance(goal_points_[count_][0], goal_points_[count_][1], vector2d);
            if(distance_ > goal_tolerance_){
                goal_stamped_.header.stamp = get_clock()->now();
                goal_stamped_.header.frame_id = "map";
                goal_stamped_.pose.position.x = goal_points_[count_][0];
                goal_stamped_.pose.position.y = goal_points_[count_][1];
                goal_stamped_.pose.position.z = goal_points_[count_][2];
                goal_stamped_.pose.orientation.x = goal_points_[count_][3];
                goal_stamped_.pose.orientation.y = goal_points_[count_][4];
                goal_stamped_.pose.orientation.z = goal_points_[count_][5];
                goal_stamped_.pose.orientation.w = goal_points_[count_][6];
                goal_publisher_->publish(goal_stamped_);
            }
            else{
                count_++;
            }

        }

        double get_disctance(double x, double y, Vector2D point2d)
        {
            return std::hypot((x - point2d.x_), (y - point2d.y_));

        }

        int count_;
        double distance_;

        std::string file_path_;
        double goal_tolerance_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_;
        
        std::vector<std::vector<double>> goal_points_;

        YAML::Node node_;
        bool change_waypoint_flg_ = false;

        Vector2D vector2d;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveGoal>());
    rclcpp::shutdown();
    return 0;
}