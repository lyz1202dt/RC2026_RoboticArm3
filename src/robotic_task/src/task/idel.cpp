#include "task/idel.hpp"
#include "robot.hpp"
#include <chrono>

IdelTask::IdelTask(Robot* context,const std::string name) : BaseTask(context,name)
{
    declare_parameters_if_needed();
}

IdelTask::~IdelTask()
{

}

void IdelTask::declare_parameters_if_needed()
{
    if (parameters_declared_ || !robot || !robot->node_) {
        return;
    }

    robot->node_->declare_parameter<std::string>("idel.next_task", "idel");
    parameters_declared_ = true;
}

std::string IdelTask::process(const std::string last_task_name)
{
    (void)last_task_name;
    declare_parameters_if_needed();

    auto node = robot->node_;
    if (node) {
        const std::string next_task = node->get_parameter("idel.next_task").as_string();
        if (next_task == "servo_demo") {
            node->set_parameter(rclcpp::Parameter("idel.next_task", "idel"));
            if (!robot->switch_motion_mode(Robot::MotionMode::SERVO)) {
                RCLCPP_WARN(node->get_logger(), "idel 无法切入 servo_demo，因为当前不处于 IDEL 模式");
                return "idel";
            }

            RCLCPP_INFO(node->get_logger(), "idel 切换到 servo_demo 调试状态");
            robot->servo_->start();
            return "servo_demo";
        }
    }

    if (!robot->wait_for_idle_signal(std::chrono::milliseconds(100))) {
        return "idel";
    }

    Robot::PendingTaskRequest request;
    if (!robot->take_pending_task(request)) {
        return "idel";
    }

    switch (request.action_type) {
        case Robot::ROBOTIC_ARM_TASK_MOVE:
            robot->finish_current_task(request.goal_handle, false, "移动任务分支尚未实现");
            break;
        case Robot::ROBOTIC_ARM_TASK_CATCH_TARGET:
            RCLCPP_INFO(node->get_logger(), "idel 分发到 catch_kfs 抓取任务");
            return "catch_kfs";
        case Robot::ROBOTIC_ARM_TASK_PLACE_TARGET:
            robot->finish_current_task(request.goal_handle, false, "放置任务分支尚未实现");
            break;
        default:
            robot->finish_current_task(request.goal_handle, false, "未知的 action_type，无法分发任务");
            break;
    }

    return "idel";
}
