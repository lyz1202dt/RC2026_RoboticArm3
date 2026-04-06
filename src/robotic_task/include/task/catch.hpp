
#pragma once

#include "task/base_task.hpp"

class CatchTask :public BaseTask{
public:
    CatchTask(Robot* context,const std::string name);
    ~CatchTask() override;
    std::string process(const std::string last_task_name) override;

private:
    /**
     * @brief 声明任务参数
     * @details 该函数用于声明任务所需的参数，如果参数已声明则不会重复声明。
     */
    void declare_parameters_if_needed();


    /**
     * @brief 标记参数是否已声明
     * @details 该变量用于记录任务参数是否已经声明，避免重复声明。
     */
    bool parameters_declared_{false};
};
