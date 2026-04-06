#pragma once

#include "robot.hpp"
#include <string>

class Robot;

/** 
 * @brief 基础任务类 
 * @details 该类为所有具体任务的基类，定义了任务的基本接口和成员变量。
 *          每个具体任务需要继承该类并实现process方法，以完成特定的任务逻辑。
 * @param robot 机器人上下文，提供对机器人状态和功能的访问。
 * @param task_name 任务名称，用于标识不同的任务。
 */
class BaseTask{
public:
    BaseTask(Robot* context,const std::string name): robot(context),task_name(name){}
    virtual ~BaseTask() = default;

    /**
     * @brief 处理任务
     * @param last_task_name 上一个任务的名称
     * @return 返回下一个要执行的任务名称
     */
    virtual std::string process(const std::string last_task_name)=0;


    Robot* robot; /* Robot 类的指针，允许任务访问机器人状态和功能 */         
                 

    std::string task_name;
};
