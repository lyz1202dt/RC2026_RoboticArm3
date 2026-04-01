#include "task/servo_demo.hpp"
#include "robot.hpp"
#include <chrono>
#include <thread>

ServoDemoTask::ServoDemoTask(Robot* context, const std::string name) : BaseTask(context, name)
{
    declare_parameters_if_needed();
}

ServoDemoTask::~ServoDemoTask()
{
}

void ServoDemoTask::declare_parameters_if_needed()
{
    if (parameters_declared_ || !robot || !robot->node_) {
        return;
    }

    auto node = robot->node_;
    node->declare_parameter<bool>("servo_demo.enable", false);
    node->declare_parameter<bool>("servo_demo.exit_to_idel", false);
    node->declare_parameter<double>("servo_demo.linear_x", 0.0);
    node->declare_parameter<double>("servo_demo.linear_y", 0.0);
    node->declare_parameter<double>("servo_demo.linear_z", 0.0);
    node->declare_parameter<double>("servo_demo.angular_x", 0.0);
    node->declare_parameter<double>("servo_demo.angular_y", 0.0);
    node->declare_parameter<double>("servo_demo.angular_z", 0.0);
    parameters_declared_ = true;
}

void ServoDemoTask::reset_command_parameters()
{
    auto node = robot->node_;
    if (!node) {
        return;
    }

    node->set_parameter(rclcpp::Parameter("servo_demo.enable", false));
    node->set_parameter(rclcpp::Parameter("servo_demo.exit_to_idel", false));
    node->set_parameter(rclcpp::Parameter("servo_demo.linear_x", 0.0));
    node->set_parameter(rclcpp::Parameter("servo_demo.linear_y", 0.0));
    node->set_parameter(rclcpp::Parameter("servo_demo.linear_z", 0.0));
    node->set_parameter(rclcpp::Parameter("servo_demo.angular_x", 0.0));
    node->set_parameter(rclcpp::Parameter("servo_demo.angular_y", 0.0));
    node->set_parameter(rclcpp::Parameter("servo_demo.angular_z", 0.0));
}

std::string ServoDemoTask::process(const std::string last_task_name)
{
    (void)last_task_name;
    declare_parameters_if_needed();

    auto node = robot->node_;
    if (!node) {
        return "idel";
    }

    if (node->get_parameter("servo_demo.exit_to_idel").as_bool()) {
        geometry_msgs::msg::Twist stop_velocity;
        (void)robot->set_arm_velocity(stop_velocity);
        reset_command_parameters();
        RCLCPP_INFO(node->get_logger(), "servo_demo 收到退出请求，返回 idel");
        return "idel";
    }

    if (!node->get_parameter("servo_demo.enable").as_bool()) {
        geometry_msgs::msg::Twist stop_velocity;
        (void)robot->set_arm_velocity(stop_velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return "servo_demo";
    }

    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = node->get_parameter("servo_demo.linear_x").as_double();
    velocity.linear.y = node->get_parameter("servo_demo.linear_y").as_double();
    velocity.linear.z = node->get_parameter("servo_demo.linear_z").as_double();
    velocity.angular.x = node->get_parameter("servo_demo.angular_x").as_double();
    velocity.angular.y = node->get_parameter("servo_demo.angular_y").as_double();
    velocity.angular.z = node->get_parameter("servo_demo.angular_z").as_double();

    const int servo_result = robot->set_arm_velocity(velocity);
    switch (servo_result) {
        case 1:
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "servo_demo: 机械臂接近奇异位置，MoveIt Servo 正在限速");
            break;
        case -1:
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "servo_demo: MoveIt Servo 报告奇异位置、碰撞或不可达，建议调整速度方向");
            break;
        case -2:
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "servo_demo: MoveIt Servo 数据暂不可用，请检查状态和 TF");
            break;
        default:
            break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(34));
    return "servo_demo";
}
