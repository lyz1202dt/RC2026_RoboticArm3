#pragma once

#include "task/base_task.hpp"

class CatchKFS :public BaseTask{
public:
    CatchKFS(Robot* context,const std::string name);
    ~CatchKFS() override;
    std::string process(const std::string last_task_name) override;

private:

};
