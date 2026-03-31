#include "robot.hpp"

#include <chrono>

Robot::Robot(const rclcpp::Node::SharedPtr node) {
    using namespace std::chrono_literals;

    node_           = node;

    //机械臂任务-动作服务客户端
    task_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task",
        std::bind(&Robot::on_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Robot::on_cancel_goal, this, std::placeholders::_1), std::bind(&Robot::on_handle_accepted, this, std::placeholders::_1)
    );


    // TF变换
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Moveit规划与控制接口
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    planning_scene_monitor_  = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    move_group_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/move_group");
    move_group_param_timer_ = node_->create_wall_timer(1s, [this]() {
        if (!move_group_param_client_->service_is_ready()) {
            return;
        }

        move_group_param_client_->set_parameters(
            {rclcpp::Parameter("trajectory_execution.allowed_start_tolerance", 0.1)});
        RCLCPP_INFO(node_->get_logger(),"重新设置轨迹执行起始容差");
        move_group_param_timer_->cancel();
    });

    // MoveIt Servo初始化
    try {
        // 获取 Servo 参数
        servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node);
        
        if (servo_parameters_) {
            // 创建 Servo 实例
            servo_ = std::make_shared<moveit_servo::Servo>(node, servo_parameters_, planning_scene_monitor_);
            servo_->start();
            
            RCLCPP_INFO(node->get_logger(), "MoveIt Servo 初始化完成");
        } else {
            RCLCPP_WARN(node->get_logger(), "加载 Servo 参数失败");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "初始化MoveIt Servo失败: %s", e.what());
    }
}

Robot::~Robot() {}


rclcpp_action::GoalResponse Robot::on_handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Robot::on_cancel_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Robot::on_handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
}
