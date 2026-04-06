#include "task/idel.hpp"
#include "robot.hpp"
#include <chrono>

IdelTask::IdelTask(Robot* context, const std::string name) : BaseTask(context, name)
{
    declare_parameters_if_needed();
}

IdelTask::~IdelTask()
{

}


/**
 * @brief 声明任务所需的ROS参数
 * 
 * @details 该函数用于声明空闲任务(IdelTask)运行所需的ROS参数。
 *          采用惰性初始化策略，仅在首次调用且满足条件时声明参数，避免重复声明。
 *          主要声明下一个任务的配置参数，用于任务切换控制。
 * 
 * @note 该函数无参数和返回值
 * @note 如果参数已声明或robot/node未初始化，则直接返回
 */
void IdelTask::declare_parameters_if_needed()
{
    // 检查是否已声明参数或必要对象未初始化，避免重复声明和空指针访问
    if (parameters_declared_ || !robot || !robot->node_) {
        return;
    }

     /**
     * 声明空闲任务的下一个任务名称参数
     * 参数名: "idel.next_task"
     * 类型: std::string
     * 默认值: "idel" (表示保持空闲状态，不切换到其他任务)
     * 用途: 允许外部通过ROS参数动态控制任务切换，例如设置为"servo_demo"可进入调试模式
     */
    robot->node_->declare_parameter<std::string>("idel.next_task", "idel");
    parameters_declared_ = true;
}


/**
 * @brief 空闲任务处理函数
 * 
 * @details 该函数处理空闲任务，执行以下步骤：
 *          1. 创建移动组接口对象
 *          2. 设置起始位置为"start_pos_3"
 *          3. 尝试规划起始位置到"start_pos_3"的路径
 *          4. 尝试执行规划的路径
 *          5. 如果规划失败，则循环尝试规划，直到成功或尝试次数达到最大值
 *          6. 如果规划成功，执行移动；如果规划失败，保持空闲状态并返回"idel"
 *          7. 声明任务参数（如果尚未声明）
 *          8. 检查ROS参数"idel.next_task"，如果设置为特定任务则切换到该任务
 *          9. 等待外部任务请求，如果收到请求则根据请求类型切换到相应任务
 */
std::string IdelTask::process(const std::string last_task_name)
{

    (void)last_task_name;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot->move_group_interface->setNamedTarget("start_pos_3");

    bool success = robot->move_group_interface->plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
    int count = 0;
    const int max_count = 100;

    while (!success && count < max_count) {
        RCLCPP_WARN(robot->node_->get_logger(), "start_pos_3 规划失败，正在重试... (尝试次数: %d)", count + 1);
        success = robot->move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        count++;
    }

    if (success) {
        RCLCPP_INFO(robot->node_->get_logger(), "start_pos_3 规划成功，执行移动");
    } else {
        RCLCPP_WARN(robot->node_->get_logger(), "start_pos_3 规划失败，保持空闲状态");
        return "idel";
    }

    do {
        robot->move_group_interface->execute(plan);
    } while (success);

    declare_parameters_if_needed();

    auto node = robot->node_;
    if (node) { 
        const std::string next_task = node->get_parameter("idel.next_task").as_string();
        if (next_task == "servo_demo") {
            node->set_parameter(rclcpp::Parameter("idel.next_task", "idel"));
            RCLCPP_INFO(node->get_logger(), "idel 切换到 servo_demo 调试状态");
            robot->servo_->start();
            return "servo_demo";
        } 
        if (next_task != "idel") {
            node->set_parameter(rclcpp::Parameter("idel.next_task", "idel"));
            RCLCPP_INFO(node->get_logger(), "idel 切换到 %s", next_task.c_str());
            return next_task;
        }
    }

    // 等待外部任务请求
    if (!robot->wait_for_idle_signal(std::chrono::milliseconds(100))) {
        return "idel";
    }

    // 尝试获取待处理的任务请求，如果获取失败则继续保持空闲状态
    Robot::PendingTaskRequest request;
    if (!robot->take_pending_task(request)) {
        return "idel";
    }

    switch (request.action_type) {
        case Robot::ROBOTIC_ARM_TASK_MOVE:
        {
            robot->finish_current_task(request.goal_handle, true, "成功接收移动任务请求，切换到 move 任务");
            return "move";
        }
        case Robot::ROBOTIC_ARM_TASK_CATCH_TARGET:
        {
            robot->finish_current_task(request.goal_handle, true, "成功接收捕获任务请求，切换到 catch 任务");
            return "catch";
        }
        case Robot::ROBOTIC_ARM_TASK_PLACE_TARGET:
        {
            robot->finish_current_task(request.goal_handle, true, "成功接收放置任务请求，切换到 place 任务");
            return "place";
        }

        default:
            robot->finish_current_task(request.goal_handle, false, "未知的 action_type，无法分发任务");
            break;
    }

    return "idel";
}
