#include "task/servo_demo.hpp"
#include "robot.hpp"
#include <chrono>
#include <thread>

/**
 * @brief ServoDemoTask 构造函数
 * 
 * @details 初始化伺服演示任务，在构造时立即声明所需的 ROS2 参数。
 *          该任务用于通过 ROS2 参数实时控制机械臂末端执行器的线速度和角速度，
 *          支持笛卡尔空间的手动操控和调试。
 * 
 * @param context 机器人上下文对象指针，提供对机器人状态和控制接口的访问
 * @param name 任务名称标识符
 */
ServoDemoTask::ServoDemoTask(Robot* context, const std::string name) : BaseTask(context, name)
{
    declare_parameters_if_needed();
}

/**
 * @brief ServoDemoTask 析构函数
 * 
 * @details 清理任务资源。当前实现为空，因为主要资源由 Robot 对象管理。
 */
ServoDemoTask::~ServoDemoTask()
{
}

/**
 * @brief 声明伺服演示任务所需的 ROS2 参数
 * 
 * @details 采用惰性初始化策略声明以下参数：
 *          - servo_demo.enable: 是否启用伺服控制（默认 false）
 *          - servo_demo.exit_to_idel: 是否退出到空闲状态（默认 false）
 *          - servo_demo.linear_x/y/z: 末端执行器线速度分量（单位：m/s，默认 0.0）
 *          - servo_demo.angular_x/y/z: 末端执行器角速度分量（单位：rad/s，默认 0.0）
 *          
 *          这些参数允许外部节点通过 ROS2 参数服务动态控制机械臂运动，
 *          适用于遥操作、调试和手动示教等场景。
 * 
 * @note 如果参数已声明或 robot/node 未初始化，则直接返回以避免重复声明
 * @note 参数声明是幂等的，多次调用不会产生副作用
 */
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

/**
 * @brief 重置所有命令参数为默认值
 * 
 * @details 将所有运动控制参数恢复为初始状态：
 *          - 禁用伺服控制（enable = false）
 *          - 清除退出标志（exit_to_idel = false）
 *          - 将所有速度分量设为零
 *          
 *          通常在任务退出或发生错误时调用，确保机械臂停止运动且参数处于安全状态。
 * 
 * @note 如果 node 未初始化则直接返回
 * @note 此操作会立即生效，外部节点需要重新设置参数才能继续控制
 */
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

/**
 * @brief 处理伺服演示任务的主循环
 * 
 * @details 该函数实现伺服控制的核心逻辑，按以下步骤执行：
 *          1. 检查退出请求：如果 exit_to_idel 为 true，停止机械臂并返回 "idel"
 *          2. 检查使能状态：如果 enable 为 false，发送零速度并短暂休眠后继续循环
 *          3. 读取速度命令：从 ROS2 参数中获取线速度和角速度的六个分量
 *          4. 执行伺服控制：调用 robot->set_arm_velocity() 发送速度指令
 *          5. 处理异常情况：根据返回值记录奇异位置、碰撞等警告信息
 *          6. 控制循环频率：休眠 34ms 以维持约 30Hz 的控制频率
 * 
 * @param last_task_name 上一个执行的任务名称（当前未使用）
 * @return 下一个要执行的任务名称，通常为 "servo_demo" 保持循环，或 "idel" 退出
 * 
 * @note 控制频率约为 30Hz（34ms 周期），适合实时手动操控
 * @note 使用 RCLCPP_WARN_THROTTLE 限制警告日志频率，避免刷屏
 * @note 异常情况下不会自动退出，需要外部设置 exit_to_idel 参数
 * 
 * @retval "servo_demo" 继续执行伺服控制任务
 * @retval "idel" 收到退出请求，切换到空闲任务
 */
std::string ServoDemoTask::process(const std::string last_task_name)
{
    (void)last_task_name;
    declare_parameters_if_needed();

    auto node = robot->node_;
    if (!node) {
        return "idel";
    }

    // 检查退出请求：优先处理退出信号以确保安全
    if (node->get_parameter("servo_demo.exit_to_idel").as_bool()) {
        geometry_msgs::msg::Twist stop_velocity;
        (void)robot->set_arm_velocity(stop_velocity);
        reset_command_parameters();
        RCLCPP_INFO(node->get_logger(), "servo_demo 收到退出请求，返回 idel");
        return "idel";
    }

    // 检查使能状态：未使能时发送零速度并保持任务活跃
    if (!node->get_parameter("servo_demo.enable").as_bool()) {
        geometry_msgs::msg::Twist stop_velocity;
        (void)robot->set_arm_velocity(stop_velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return "servo_demo";
    }

    // 从 ROS2 参数中读取六维速度命令
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = node->get_parameter("servo_demo.linear_x").as_double();
    velocity.linear.y = node->get_parameter("servo_demo.linear_y").as_double();
    velocity.linear.z = node->get_parameter("servo_demo.linear_z").as_double();
    velocity.angular.x = node->get_parameter("servo_demo.angular_x").as_double();
    velocity.angular.y = node->get_parameter("servo_demo.angular_y").as_double();
    velocity.angular.z = node->get_parameter("servo_demo.angular_z").as_double();

    // 执行伺服控制并处理返回状态
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

    // 控制循环频率：34ms 对应约 30Hz，平衡响应速度和系统负载
    std::this_thread::sleep_for(std::chrono::milliseconds(34));
    return "servo_demo";
}