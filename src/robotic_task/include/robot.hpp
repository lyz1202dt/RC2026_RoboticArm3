#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_scene_interface.h>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/servo_parameters.hpp>
#include <robot_interfaces/action/catch.hpp>

class Robot{
    public:
    Robot(const rclcpp::Node::SharedPtr node);
    ~Robot();
    private:

    rclcpp_action::GoalResponse on_handle_goal(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal);
    rclcpp_action::CancelResponse on_cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);
    void on_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);



    rclcpp::Node::SharedPtr node_;
    rclcpp::AsyncParametersClient::SharedPtr param_client;
    std::shared_ptr<std::thread> arm_task_thread;
    rclcpp_action::Server<robot_interfaces::action::Catch>::SharedPtr arm_handle_server;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
    
    // MoveIt Servo
    std::shared_ptr<moveit_servo::Servo> servo_;
    std::shared_ptr<moveit_servo::ServoParameters> servo_parameters_;
};
