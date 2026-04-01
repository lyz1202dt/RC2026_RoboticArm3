#pragma once

#include "task/base_task.hpp"

class ServoDemoTask : public BaseTask {
public:
    ServoDemoTask(Robot* context, const std::string name);
    ~ServoDemoTask() override;
    std::string process(const std::string last_task_name) override;

private:
    void declare_parameters_if_needed();
    void reset_command_parameters();
    bool parameters_declared_{false};
};
