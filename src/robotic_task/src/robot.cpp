#include "robot.hpp"
#include "task/base_task.hpp"
#include "task/idel.hpp"
#include "task/servo_demo.hpp"
#include <chrono>
#include <thread>
#include <utility>

/**
 * @brief Robot 类构造函数
 * 
 * @details 初始化机器人控制系统的核心组件，包括：
 *          1. ROS2 节点和发布器（气泵控制）
 *          2. Action 服务器（机械臂任务接口）
 *          3. TF 变换监听器
 *          4. MoveIt 规划与控制接口
 *          5. MoveIt Servo 实时速度控制
 *          6. 任务注册与调度器初始化
 *          7. 启动任务调度线程
 * 
 * @param node ROS2 节点共享指针，用于创建通信接口和资源管理
 */
Robot::Robot(const rclcpp::Node::SharedPtr node) {
    using namespace std::chrono_literals;

    node_ = node;
    
    // 创建气泵控制发布器，用于控制吸盘的开启/关闭
    air_pump_command_pub_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/suction_controller/commands", rclcpp::SystemDefaultsQoS());

    // 创建机械臂任务 Action 服务器
    // 服务名: "robotic_task"
    // 处理三个回调：目标接收、取消请求、目标接受
    task_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task", 
        std::bind(&Robot::on_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Robot::on_cancel_goal, this, std::placeholders::_1), 
        std::bind(&Robot::on_handle_accepted, this, std::placeholders::_1));


    // 初始化 TF 变换缓冲区和监听器，用于坐标系转换
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // 初始化 MoveIt 规划与控制相关接口
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    planning_scene_monitor_  = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    
    // 启动规划场景监控，实时获取机器人状态、场景信息和世界几何体
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    // 配置 MoveGroup 参数：设置轨迹执行起始容差为 0.1
    // 使用定时器在 MoveGroup 服务就绪后一次性设置参数
    move_group_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/move_group");
    move_group_param_timer_  = node_->create_wall_timer(1s, [this]() {
        if (!move_group_param_client_->service_is_ready()) {
            return;
        }

        move_group_param_client_->set_parameters({rclcpp::Parameter("trajectory_execution.allowed_start_tolerance", 0.1)});
        RCLCPP_INFO(node_->get_logger(), "重新设置轨迹执行起始容差");
        move_group_param_timer_->cancel();  // 参数设置完成后取消定时器
    });

    // 初始化 MoveIt Servo 实时速度控制器
    try {
        // 从 ROS2 参数服务器加载 Servo 配置参数
        servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node);

        if (servo_parameters_) {
            // 创建 Servo 实例，传入参数和规划场景监控器
            servo_ = std::make_shared<moveit_servo::Servo>(node, servo_parameters_, planning_scene_monitor_);
            
            // 创建速度命令发布器，话题名由参数配置
            servo_twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                servo_parameters_->cartesian_command_in_topic, rclcpp::SystemDefaultsQoS());
            
            // 订阅 Servo 状态反馈，用于判断控制结果和异常情况
            servo_status_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
                servo_parameters_->status_topic, rclcpp::SystemDefaultsQoS(),
                [this](const std_msgs::msg::Int8::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(servo_status_mutex_);
                    latest_servo_status_ = static_cast<moveit_servo::StatusCode>(msg->data);
                    latest_servo_status_stamp_ = node_->now();
                    servo_status_received_ = true;
                });
            RCLCPP_INFO(node->get_logger(), "MoveIt Servo 初始化完成");
        } else {
            RCLCPP_WARN(node->get_logger(), "加载 Servo 参数失败");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "初始化MoveIt Servo失败: %s", e.what());
    }

    // 注册任务到任务表
    register_task(std::make_shared<IdelTask>(this,"idel"));
    register_task(std::make_shared<ServoDemoTask>(this,"servo_demo"));
    
    // 初始化任务调度器，设置初始任务为 "idel"
    init_task_manager("idel");
    
    // 启动任务调度线程，持续执行当前任务并根据返回值切换任务
    arm_task_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            porcess_task();
        }
    });
}

/**
 * @brief Robot 类析构函数
 * 
 * @details 等待任务调度线程安全退出，确保资源正确释放
 */
Robot::~Robot() {
    if (arm_task_thread->joinable())
        arm_task_thread->join();
}


/**
 * @brief Action 目标接收回调 - 决定是否接受新目标
 * 
 * @details 检查当前是否有任务正在执行或等待调度：
 *          - 如果有，则拒绝新目标并记录警告
 *          - 如果没有，则保存目标信息并标记为待处理状态
 * 
 * @param uuid 目标的唯一标识符（当前未使用）
 * @param goal 目标对象，包含动作类型和目标位姿
 * @return GoalResponse ACCEPT_AND_EXECUTE 表示接受目标，REJECT 表示拒绝
 */
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

/**
 * @brief Action 目标取消回调
 * 
 * @details 接受取消请求。实际的取消逻辑在任务执行过程中处理。
 * 
 * @param goal_handle 目标句柄（当前未使用）
 * @return CancelResponse ACCEPT 表示接受取消请求
 */
rclcpp_action::CancelResponse
    Robot::on_cancel_goal(const std::shared_ptr<CatchGoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Action 目标接受回调
 * 
 * @details 当目标被接受后调用，保存目标句柄并通知空闲任务开始分发。
 *          通过信号量机制唤醒空闲任务的处理循环。
 * 
 * @param goal_handle 已接受的目标句柄
 */
void Robot::on_handle_accepted(const std::shared_ptr<CatchGoalHandle> goal_handle) {
    bool should_notify_idel = false;
    {
        std::lock_guard<std::mutex> lock(action_state_mutex_);
        pending_goal_handle_ = goal_handle;
        should_notify_idel = !task_executing_ && goal_pending_;
    }

    if (should_notify_idel) {
        RCLCPP_INFO(node_->get_logger(), "目标已接受，通知 idel 任务开始分发");
        idle_task_signal_.release();  // 释放信号量，唤醒等待的空闲任务
    }
}

/**
 * @brief 等待空闲任务信号
 * 
 * @details 阻塞等待直到收到新目标通知或超时。
 *          用于空闲任务中等待外部任务请求。
 * 
 * @param timeout 超时时间
 * @return true 收到信号，false 超时
 */
bool Robot::wait_for_idle_signal(const std::chrono::milliseconds timeout) {
    return idle_task_signal_.try_acquire_for(timeout);
}

/**
 * @brief 获取待处理的任务请求
 * 
 * @details 从内部状态中提取待处理的目标信息，并更新任务执行状态。
 *          此操作会将目标从"待处理"状态转换为"执行中"状态。
 * 
 * @param request 输出参数，填充任务请求信息（动作类型、目标位姿、目标句柄）
 * @return true 成功获取待处理任务，false 没有待处理任务
 */
bool Robot::take_pending_task(PendingTaskRequest& request) {
    std::lock_guard<std::mutex> lock(action_state_mutex_);
    if (!goal_pending_ || !pending_goal_handle_) {
        return false;
    }

    request.action_type = expected_action_type_;
    request.target_pose = expected_target_pose_;
    request.goal_handle = pending_goal_handle_;

    goal_pending_ = false;   // 清除待处理标志
    task_executing_ = true;  // 标记任务开始执行
    return true;
}

/**
 * @brief 完成当前任务
 * 
 * @details 根据执行结果向 Action 客户端返回成功或失败响应，
 *          并重置任务执行状态以允许接收新目标。
 * 
 * @param goal_handle 目标句柄，用于发送结果
 * @param success 任务是否成功执行
 * @param reason 任务完成或失败的原因描述
 */
void Robot::finish_current_task(const std::shared_ptr<CatchGoalHandle>& goal_handle, const bool success, const std::string& reason) {
    auto result = std::make_shared<Catch::Result>();
    result->reason = reason;
    result->kfs_num = 0;

    {
        std::lock_guard<std::mutex> lock(action_state_mutex_);
        task_executing_ = false;  // 清除执行标志，允许接收新目标

        if (pending_goal_handle_ == goal_handle) {
            pending_goal_handle_.reset();  // 清空待处理句柄
        }
    }

    if (!goal_handle) {
        RCLCPP_WARN(node_->get_logger(), "任务结束时 goal_handle 为空");
        return;
    }

    if (success) {
        goal_handle->succeed(result);  // 标记目标成功
    } else {
        goal_handle->abort(result);    // 标记目标中止
    }
}


/**
 * @brief 任务调度主循环
 * 
 * @details 该函数在独立线程中持续运行，负责：
 *          1. 等待任务管理器初始化完成
 *          2. 获取当前任务并执行其 process() 方法
 *          3. 根据返回值切换到下一个任务
 *          4. 处理异常情况和无效的任务切换请求
 *          
 *          任务切换逻辑：
 *          - 如果返回空字符串，结束当前任务序列
 *          - 如果返回未注册的任务名，报错并保持等待
 *          - 如果返回有效任务名，切换到该任务并继续执行
 * 
 * @note 使用条件变量等待任务管理器就绪，避免忙等待
 * @note 包含完整的异常捕获，确保单个任务异常不会导致调度器崩溃
 */
void Robot::porcess_task(){
    std::shared_ptr<BaseTask> current_task;
    std::string current_task_name;
    std::string previous_task_name;

    {
        // 等待任务管理器初始化且当前任务名非空
        std::unique_lock<std::mutex> lock(task_manager_mutex_);
        task_manager_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
            return !rclcpp::ok() || (task_manager_initialized_ && !current_task_name_.empty());
        });

        if (!rclcpp::ok()) {
            return;
        }

        // 查找当前任务
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

    // 执行当前任务的 process 方法
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

    // 处理任务切换逻辑
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

/**
 * @brief 注册任务到任务表
 * 
 * @details 将任务对象添加到任务表中，支持任务名的唯一性检查。
 *          如果任务名已存在，则更新为新的任务实现。
 *          注册完成后通知等待的条件变量。
 * 
 * @param task_ptr 任务对象的共享指针
 * 
 * @note 如果 task_ptr 为空或任务名为空，会记录错误并返回
 * @note 使用 insert_or_assign 确保任务名的唯一性
 */
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

    task_manager_cv_.notify_all();  // 通知所有等待的线程
}

/**
 * @brief 初始化任务调度器
 * 
 * @details 设置任务调度器的初始状态，包括：
 *          1. 验证是否有已注册的任务
 *          2. 设置初始任务名称（如果未指定则使用首个注册任务）
 *          3. 验证初始任务是否已注册
 *          4. 标记调度器为已初始化状态
 *          5. 通知等待的条件变量启动调度循环
 * 
 * @param first_task_name 初始任务名称，如果为空则自动选择首个注册任务
 * 
 * @note 必须在注册至少一个任务后才能调用
 * @note 如果初始任务未注册，调度器将保持未初始化状态
 */
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
    task_manager_cv_.notify_all();  // 唤醒任务调度线程
}

/**
 * @brief 控制气泵开关
 * 
 * @details 向气泵控制器发布命令消息，控制吸盘的开启或关闭。
 *          使用 control_msgs::msg::DynamicInterfaceGroupValues 消息类型，
 *          设置 interface_groups 为 "suction"，interface_names 为 "vacuum"。
 * 
 * @param enable true 开启气泵（产生负压），false 关闭气泵
 * @return true 命令发布成功，false 发布器未初始化
 * 
 * @note 消息中包含时间戳，用于同步和调试
 * @note 真空值：1.0 表示开启，0.0 表示关闭
 */
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

/**
 * @brief 设置机械臂末端执行器的笛卡尔空间速度
 * 
 * @details 通过 MoveIt Servo 接口发送六维速度命令（线速度 + 角速度），
 *          并等待 Servo 状态反馈以判断命令执行情况。
 *          
 *          执行流程：
 *          1. 验证 Servo 控制链路完整性
 *          2. 检查 TF 变换数据可用性
 *          3. 构造并发布 TwistStamped 速度命令
 *          4. 等待 Servo 状态反馈（最多等待 2 倍发布周期）
 *          5. 根据状态码返回不同的结果
 * 
 * @param velocity 六维速度命令，包含线速度 (linear) 和角速度 (angular)
 * @return int 执行结果状态码：
 *         - 0: 成功执行，无警告
 *         - 1: 接近奇异位置，Servo 自动降速
 *         - -1: 遇到奇异位置、碰撞或关节限制，建议调整速度方向
 *         - -2: Servo 数据不可用或未收到状态反馈
 * 
 * @note 速度命令在 servo_parameters_->robot_link_command_frame 坐标系下解释
 * @note 使用轮询方式等待状态反馈，每次休眠 5ms
 * @note 如果超时未收到状态，使用最新收到的状态作为参考
 */
int Robot::set_arm_velocity(const geometry_msgs::msg::Twist &velocity)
{
    if (!node_ || !servo_ || !servo_parameters_ || !servo_twist_pub_ || !servo_status_sub_) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "MoveIt Servo 控制链路存在空指针");
        }
        return -2;
    }

    // 验证 TF 变换数据可用性
    geometry_msgs::msg::TransformStamped command_frame_transform;
    geometry_msgs::msg::TransformStamped ee_frame_transform;
    if (!servo_->getCommandFrameTransform(command_frame_transform) || !servo_->getEEFrameTransform(ee_frame_transform)) {
        RCLCPP_WARN(node_->get_logger(), "MoveIt Servo 变换数据暂不可用");
        return -2;
    }

    // 构造并发布速度命令
    geometry_msgs::msg::TwistStamped twist_command;
    twist_command.header.stamp = node_->now();
    twist_command.header.frame_id = servo_parameters_->robot_link_command_frame;
    twist_command.twist = velocity;
    servo_twist_pub_->publish(twist_command);

    // 等待 Servo 状态反馈，最多等待 2 倍发布周期
    const auto wait_deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(servo_parameters_->publish_period * 2.0);

    moveit_servo::StatusCode status = moveit_servo::StatusCode::INVALID;
    bool has_status = false;

    while (std::chrono::steady_clock::now() < wait_deadline) {
        {
            std::lock_guard<std::mutex> lock(servo_status_mutex_);
            // 检查是否收到比命令时间戳更新的状态
            has_status = servo_status_received_ && latest_servo_status_stamp_ >= twist_command.header.stamp;
            if (has_status) {
                status = latest_servo_status_;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 每 5ms 检查一次
    }

    // 如果超时，使用最新收到的状态
    if (!has_status) {
        std::lock_guard<std::mutex> lock(servo_status_mutex_);
        has_status = servo_status_received_;
        status = latest_servo_status_;
    }

    if (!has_status) {
        RCLCPP_WARN(node_->get_logger(), "尚未收到 MoveIt Servo 状态，无法判断控制结果");
        return -2;
    }

    // 根据状态码返回相应的结果
    switch (status) {
        case moveit_servo::StatusCode::NO_WARNING:
            return 0;  // 正常执行
        case moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY:
        case moveit_servo::StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY:
            return 1;  // 接近或离开奇异位置，自动降速
        case moveit_servo::StatusCode::HALT_FOR_SINGULARITY:
        case moveit_servo::StatusCode::DECELERATE_FOR_COLLISION:
        case moveit_servo::StatusCode::HALT_FOR_COLLISION:
        case moveit_servo::StatusCode::JOINT_BOUND:
        case moveit_servo::StatusCode::INVALID:
        default:
            return -1;  // 严重异常：奇异位置、碰撞、关节超限等
    }
}