#pragma once

#include <chrono>
#include <condition_variable>
#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/catch.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <unordered_map>

class BaseTask;

class Robot {
public:
    using Catch = robot_interfaces::action::Catch;
    using CatchGoal = Catch::Goal;
    using CatchGoalHandle = rclcpp_action::ServerGoalHandle<Catch>;

    class SimpleSemaphore {
    public:
        explicit SimpleSemaphore(std::size_t initial_count = 0) : count_(initial_count) {}

        void release() {
            std::lock_guard<std::mutex> lock(mutex_);
            ++count_;
            cv_.notify_one();
        }

        bool try_acquire_for(const std::chrono::milliseconds timeout) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!cv_.wait_for(lock, timeout, [this]() { return count_ > 0; })) {
                return false;
            }

            --count_;
            return true;
        }

    private:
        std::mutex mutex_;
        std::condition_variable cv_;
        std::size_t count_{0};
    };

    struct PendingTaskRequest {
        int32_t action_type{0};
        geometry_msgs::msg::Pose target_pose;
        std::shared_ptr<CatchGoalHandle> goal_handle;
    };

    typedef enum {
        ROBOTIC_ARM_TASK_MOVE         = 1, // 移动到某个位姿
        ROBOTIC_ARM_TASK_CATCH_TARGET = 2, // 捕获处于某个坐标下的KFS
        ROBOTIC_ARM_TASK_PLACE_TARGET = 3  // 将机器人上的KFS放置到某个坐标
    } ArmTask;

    Robot(const rclcpp::Node::SharedPtr node);
    ~Robot();
    
    bool wait_for_idle_signal(const std::chrono::milliseconds timeout);
    bool take_pending_task(PendingTaskRequest& request);
    void finish_current_task(const std::shared_ptr<CatchGoalHandle>& goal_handle, bool success, const std::string& reason);
    int set_arm_velocity(const geometry_msgs::msg::Twist &velocity);

    rclcpp_action::Server<Catch>::SharedPtr task_handle_server;
    rclcpp_action::GoalResponse
        on_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CatchGoal> goal);
    rclcpp_action::CancelResponse
        on_cancel_goal(const std::shared_ptr<CatchGoalHandle> goal_handle);
    void on_handle_accepted(const std::shared_ptr<CatchGoalHandle> goal_handle);


    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<std::thread> arm_task_thread;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    rclcpp::AsyncParametersClient::SharedPtr move_group_param_client_;
    rclcpp::TimerBase::SharedPtr move_group_param_timer_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr air_pump_command_pub_;

    // MoveIt Servo
    std::shared_ptr<moveit_servo::Servo> servo_;
    moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
    std::mutex servo_status_mutex_;
    moveit_servo::StatusCode latest_servo_status_{moveit_servo::StatusCode::INVALID};
    rclcpp::Time latest_servo_status_stamp_{0, 0, RCL_ROS_TIME};
    bool servo_status_received_{false};

    // 初始化任务调度器，执行任务，切换任务
    void porcess_task();
    void register_task(std::shared_ptr<BaseTask> task_ptr);
    void init_task_manager(const std::string first_task_name);

    std::mutex task_manager_mutex_;
    std::condition_variable task_manager_cv_;
    std::unordered_map<std::string, std::shared_ptr<BaseTask>> task_table_;
    std::string current_task_name_;
    std::string last_task_name_;
    bool task_manager_initialized_{false};

    std::mutex action_state_mutex_;
    bool task_executing_{false};
    bool goal_pending_{false};
    int32_t expected_action_type_{0};
    geometry_msgs::msg::Pose expected_target_pose_;
    std::shared_ptr<CatchGoalHandle> pending_goal_handle_;
    SimpleSemaphore idle_task_signal_;


    // 工具函数
    bool set_air_pump(const bool enable);
};
