#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <functional>
#include <memory>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <robot_interfaces/action/catch.hpp>

class Robot{
    public:
    Robot(const rclcpp::Node::SharedPtr node);
    ~Robot();
    private:

    rclcpp_action::Server<robot_interfaces::action::Catch>::SharedPtr task_handle_server;
    rclcpp_action::GoalResponse on_handle_goal(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal);
    rclcpp_action::CancelResponse on_cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);
    void on_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);


    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<std::thread> arm_task_thread;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    rclcpp::AsyncParametersClient::SharedPtr move_group_param_client_;
    rclcpp::TimerBase::SharedPtr move_group_param_timer_;

    // MoveIt Servo
    std::shared_ptr<moveit_servo::Servo> servo_;
    moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
};
