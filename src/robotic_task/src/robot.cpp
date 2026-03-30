#include "robot.hpp"

Robot::Robot(const rclcpp::Node::SharedPtr node) {
    node_           = node;
    param_client    = std::make_shared<rclcpp::AsyncParametersClient>(node, "driver_node");
    arm_task_thread = nullptr; // 延后创建线程，确保构造完成后再启动



    // 任务动作
    arm_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task", // 创建动作服务-服务端
        std::bind(&Robot::on_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Robot::on_cancel_goal, this, std::placeholders::_1), std::bind(&Robot::on_handle_accepted, this, std::placeholders::_1)
    );


    // TF变换
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Moveit规划与控制接口
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // MoveIt Servo初始化
    try {
        // 获取 Servo 参数
        servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node);
        
        if (servo_parameters_) {
            // 创建 Servo 实例
            servo_ = std::make_shared<moveit_servo::Servo>(node, servo_parameters_, planning_scene_interface);
            
            RCLCPP_INFO(node->get_logger(), "MoveIt Servo initialized successfully");
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to load Servo parameters");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MoveIt Servo: %s", e.what());
    }
}

Robot::~Robot() {}


rclcpp_action::GoalResponse on_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal) {

}

rclcpp_action::CancelResponse on_cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {

}

void on_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {

}
