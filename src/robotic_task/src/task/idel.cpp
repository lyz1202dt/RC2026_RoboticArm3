#include "task/idel.hpp"
#include "robot.hpp"
#include <chrono>

IdelTask::IdelTask(Robot* context,const std::string name) : BaseTask(context,name)
{

}

IdelTask::~IdelTask()
{

}

std::string IdelTask::process(const std::string last_task_name)
{
    (void)last_task_name;

    if (!robot->wait_for_idle_signal(std::chrono::milliseconds(100))) {
        return "idel";
    }

    Robot::PendingTaskRequest request;
    if (!robot->take_pending_task(request)) {
        return "idel";
    }

    switch (request.action_type) {
        case Robot::ROBOTIC_ARM_TASK_MOVE:
        case Robot::ROBOTIC_ARM_TASK_CATCH_TARGET:
        case Robot::ROBOTIC_ARM_TASK_PLACE_TARGET:
            robot->finish_current_task(request.goal_handle, false, "任务已进入idel分发框架，但具体子任务尚未实现");
            break;
        default:
            robot->finish_current_task(request.goal_handle, false, "未知的 action_type，无法分发任务");
            break;
    }

    return "idel";
}
