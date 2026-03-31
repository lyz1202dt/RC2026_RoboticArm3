#include "robot.hpp"
#include "task/base_task.hpp"
#include "task/idel.hpp"
#include <chrono>
#include <utility>

Robot::Robot(const rclcpp::Node::SharedPtr node) {
    using namespace std::chrono_literals;

    node_ = node;
    air_pump_command_pub_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/suction_controller/commands", rclcpp::SystemDefaultsQoS());

    // 机械臂任务-动作服务客户端
    task_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task", std::bind(&Robot::on_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Robot::on_cancel_goal, this, std::placeholders::_1), std::bind(&Robot::on_handle_accepted, this, std::placeholders::_1));


    // TF变换
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Moveit规划与控制接口
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    planning_scene_monitor_  = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    move_group_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/move_group");
    move_group_param_timer_  = node_->create_wall_timer(1s, [this]() {
        if (!move_group_param_client_->service_is_ready()) {
            return;
        }

        move_group_param_client_->set_parameters({rclcpp::Parameter("trajectory_execution.allowed_start_tolerance", 0.1)});
        RCLCPP_INFO(node_->get_logger(), "重新设置轨迹执行起始容差");
        move_group_param_timer_->cancel();
    });

    // MoveIt Servo初始化
    try {
        // 获取 Servo 参数
        servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node);

        if (servo_parameters_) {
            // 创建 Servo 实例
            servo_ = std::make_shared<moveit_servo::Servo>(node, servo_parameters_, planning_scene_monitor_);
            servo_->start();

            RCLCPP_INFO(node->get_logger(), "MoveIt Servo 初始化完成");
        } else {
            RCLCPP_WARN(node->get_logger(), "加载 Servo 参数失败");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "初始化MoveIt Servo失败: %s", e.what());
    }

    register_task(std::make_shared<IdelTask>(this,"idel"));
    init_task_manager("idel");
    arm_task_thread = std::make_shared<std::thread>([this]() {  //任务调度线程
        while (rclcpp::ok()) {
            porcess_task();
        }
    });
}

Robot::~Robot() {
    if (arm_task_thread->joinable())
        arm_task_thread->join();
}


rclcpp_action::GoalResponse
    Robot::on_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CatchGoal> goal) {
    (void)uuid;

    std::lock_guard<std::mutex> lock(action_state_mutex_);
    if (task_executing_ || goal_pending_) {
        RCLCPP_WARN(node_->get_logger(), "当前已有任务正在执行或等待调度，拒绝新的目标");
        return rclcpp_action::GoalResponse::REJECT;
    }

    expected_action_type_ = goal->action_type;
    expected_target_pose_ = goal->target_pose;
    goal_pending_ = true;

    RCLCPP_INFO(node_->get_logger(),
                "接收到新目标: action_type=%d, target_pose=(%.3f, %.3f, %.3f)",
                expected_action_type_,
                expected_target_pose_.position.x,
                expected_target_pose_.position.y,
                expected_target_pose_.position.z);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
    Robot::on_cancel_goal(const std::shared_ptr<CatchGoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Robot::on_handle_accepted(const std::shared_ptr<CatchGoalHandle> goal_handle) {
    bool should_notify_idel = false;
    {
        std::lock_guard<std::mutex> lock(action_state_mutex_);
        pending_goal_handle_ = goal_handle;
        should_notify_idel = !task_executing_ && goal_pending_;
    }

    if (should_notify_idel) {
        RCLCPP_INFO(node_->get_logger(), "目标已接受，通知 idel 任务开始分发");
        idle_task_signal_.release();
    }
}

bool Robot::wait_for_idle_signal(const std::chrono::milliseconds timeout) {
    return idle_task_signal_.try_acquire_for(timeout);
}

bool Robot::take_pending_task(PendingTaskRequest& request) {
    std::lock_guard<std::mutex> lock(action_state_mutex_);
    if (!goal_pending_ || !pending_goal_handle_) {
        return false;
    }

    request.action_type = expected_action_type_;
    request.target_pose = expected_target_pose_;
    request.goal_handle = pending_goal_handle_;

    goal_pending_ = false;
    task_executing_ = true;
    return true;
}

void Robot::finish_current_task(const std::shared_ptr<CatchGoalHandle>& goal_handle, const bool success, const std::string& reason) {
    auto result = std::make_shared<Catch::Result>();
    result->reason = reason;
    result->kfs_num = 0;

    {
        std::lock_guard<std::mutex> lock(action_state_mutex_);
        task_executing_ = false;

        if (pending_goal_handle_ == goal_handle) {
            pending_goal_handle_.reset();
        }
    }

    if (!goal_handle) {
        RCLCPP_WARN(node_->get_logger(), "任务结束时 goal_handle 为空");
        return;
    }

    if (success) {
        goal_handle->succeed(result);
    } else {
        goal_handle->abort(result);
    }
}


void Robot::porcess_task(){
    std::shared_ptr<BaseTask> current_task;
    std::string current_task_name;
    std::string previous_task_name;

    {
        std::unique_lock<std::mutex> lock(task_manager_mutex_);
        task_manager_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
            return !rclcpp::ok() || (task_manager_initialized_ && !current_task_name_.empty());
        });

        if (!rclcpp::ok()) {
            return;
        }

        auto task_it = task_table_.find(current_task_name_);
        if (task_it == task_table_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "当前任务 [%s] 未注册，任务调度暂停", current_task_name_.c_str());
            current_task_name_.clear();
            return;
        }

        current_task = task_it->second;
        current_task_name = current_task_name_;
        previous_task_name = last_task_name_;
    }

    std::string next_task_name;
    try {
        next_task_name = current_task->process(previous_task_name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "执行任务 [%s] 时发生异常: %s", current_task_name.c_str(), e.what());
        next_task_name.clear();
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "执行任务 [%s] 时发生未知异常", current_task_name.c_str());
        next_task_name.clear();
    }

    {
        std::lock_guard<std::mutex> lock(task_manager_mutex_);
        last_task_name_ = current_task_name;

        if (next_task_name.empty()) {
            RCLCPP_INFO(node_->get_logger(), "任务 [%s] 执行完成，当前没有后续任务", current_task_name.c_str());
            current_task_name_.clear();
            return;
        }

        if (task_table_.find(next_task_name) == task_table_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "任务 [%s] 请求切换到未注册任务 [%s]，调度保持等待", current_task_name.c_str(), next_task_name.c_str());
            current_task_name_.clear();
            return;
        }

        if (next_task_name != current_task_name) {
            RCLCPP_INFO(node_->get_logger(), "任务切换: [%s] -> [%s]", current_task_name.c_str(), next_task_name.c_str());
        }
        current_task_name_ = std::move(next_task_name);
    }
}

void Robot::register_task(std::shared_ptr<BaseTask> task_ptr){
    if (!task_ptr) {
        RCLCPP_ERROR(node_->get_logger(), "注册任务失败: task_ptr 为空");
        return;
    }

    const std::string task_name = task_ptr->task_name;
    if (task_name.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "注册任务失败: 任务名为空");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(task_manager_mutex_);
        auto [it, inserted] = task_table_.insert_or_assign(task_name, std::move(task_ptr));
        (void)it;
        if (inserted) {
            RCLCPP_INFO(node_->get_logger(), "注册任务成功: [%s]", task_name.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "任务 [%s] 已存在，已更新为新的任务实现", task_name.c_str());
        }
    }

    task_manager_cv_.notify_all();
}

void Robot::init_task_manager(const std::string first_task_name){
    std::lock_guard<std::mutex> lock(task_manager_mutex_);
    if (task_table_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "初始化任务调度器失败: 当前没有已注册任务");
        task_manager_initialized_ = false;
        current_task_name_.clear();
        last_task_name_.clear();
        return;
    }

    std::string initial_task_name = first_task_name;
    if (initial_task_name.empty()) {
        initial_task_name = task_table_.begin()->first;
        RCLCPP_WARN(node_->get_logger(), "未指定初始任务，默认使用已注册的首个任务 [%s]", initial_task_name.c_str());
    }

    if (task_table_.find(initial_task_name) == task_table_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "初始化任务调度器失败: 初始任务 [%s] 未注册", initial_task_name.c_str());
        task_manager_initialized_ = false;
        current_task_name_.clear();
        last_task_name_.clear();
        return;
    }

    current_task_name_ = std::move(initial_task_name);
    last_task_name_.clear();
    task_manager_initialized_ = true;

    RCLCPP_INFO(node_->get_logger(), "任务调度器初始化完成，初始任务为 [%s]", current_task_name_.c_str());
    task_manager_cv_.notify_all();
}

bool Robot::set_air_pump(const bool enable){
    if (!air_pump_command_pub_) {
        RCLCPP_ERROR(node_->get_logger(), "气泵控制发布器未初始化");
        return false;
    }

    control_msgs::msg::DynamicInterfaceGroupValues command_msg;
    command_msg.header.stamp = node_->now();
    command_msg.interface_groups.emplace_back("suction");

    control_msgs::msg::InterfaceValue interface_value;
    interface_value.interface_names.emplace_back("vacuum");
    interface_value.values.emplace_back(enable ? 1.0 : 0.0);
    command_msg.interface_values.emplace_back(std::move(interface_value));

    air_pump_command_pub_->publish(command_msg);
    RCLCPP_INFO(node_->get_logger(), "气泵已设置为: %s", enable ? "开启" : "关闭");
    return true;
}
