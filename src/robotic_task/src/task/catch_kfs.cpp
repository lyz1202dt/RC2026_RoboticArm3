#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <chrono>

CatchKFS::CatchKFS(Robot* context,const std::string name) : BaseTask(context,name)
{
    
}

CatchKFS::~CatchKFS()
{
    
}


std::string CatchKFS::process(const std::string last_task_name)
{
    (void)last_task_name;
    
    return "idel";
}
