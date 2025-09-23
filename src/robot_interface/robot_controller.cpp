#include <iostream>
#include <robot_interface/robot_controller.h>
RobotController::RobotController(std::string name, int num_joints,
                                 ControllerConfig config)
    : name_(name), num_joints_(num_joints), config_(config) {}
/**
 * @brief 初始化机器人控制器
 *
 * @return int 初始化成功返回0，否则返回-1
 */
int RobotController::Initialize(std::chrono::milliseconds timer_period) {
  // 完成机器人初始化后，获取timestamp，启动计时器
  // 子类需要实现Initialize，并在最后调用RobotController::Initialize()
  int ret = -1;
  timer_ = Timer::create(
      name_, std::bind([this]() { return this->timer_cb(); }), timer_period);
  robot_initialized_ = 1;
  ret = 0;
  return ret;
}
RobotController::RobotJointState RobotController::getJointState() {
  // Example
  RobotJointState joint_state;
  joint_state.joints = num_joints_;
  joint_state.joint_state = Eigen::VectorXd::Zero(joint_state.joints) * 2;
  joint_state.timestamp = std::chrono::steady_clock::now();
  return joint_state;
}
/**
 * @brief 基本回调函数，实现位置环控制
 *
 * @return int
 */
int RobotController::timer_cb() {
  // 从getJointState()获取当前关节状态
  static auto start = std::chrono::steady_clock::now();
  auto t = std::chrono::duration_cast<std::chrono::duration<double>>(
               std::chrono::steady_clock::now() - start)
               .count();
  // std::cout << "timer_cb:t_start = " << t << std::endl;
  auto current = getJointState();
  RobotJointState last, target, last_target;
  {
    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    last = last_joint_state_;
    target = target_joint_state_;
    last_target = last_target_joint_state_;
    current_joint_state_ = current;
  }
  // 用于调试
  int target_size = target.joint_state.rows();
  int last_target_size = last_target.joint_state.rows();
  int current_size = current.joint_state.rows();
  int last_size = last.joint_state.rows();
  // 此时target未被初始化或存在数据读取错误
  if (target_size != current_size || target_size != last_size) {
    return 1;
  }
  //此时target已经初始化，初始化last_target
  if (last_target_size != target_size) {
    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    last_target_joint_state_ = target;
    return 2;
  }
  if (last.timestamp == current.timestamp) {
    std::cout << "Timer callback: same timestamp" << std::endl;
    return 0;
  }
  // 计算状态量速度
  double delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(
                       current.timestamp - last.timestamp)
                       .count();
  // 计算关节状态的速度
  Eigen::VectorXd q_dot = (current.joint_state - last.joint_state);
  Eigen::VectorXd q_dot_target = (target.joint_state - last_target.joint_state);
  // 计算新的关节状态
  RobotJointState new_joint_state;
  new_joint_state.joints = num_joints_;
  // p_dot = k_p*(p_target - p)
  new_joint_state.joint_state =
      current.joint_state +
      config_.k_p * (target.joint_state - current.joint_state) +
      config_.k_v * (q_dot_target - q_dot);
  new_joint_state.timestamp = current.timestamp;
  // 写入
  setJointState(new_joint_state);
  // 更新上一个关节状态
  {
    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    last_joint_state_ = current;
    last_target_joint_state_ = target;
  }
  auto t_end = std::chrono::duration_cast<std::chrono::duration<double>>(
                   std::chrono::steady_clock::now() - start)
                   .count();

  // std::cout << "timer_cb:t_end = " << t_end << std::endl;
  return 0;
}
/**
 * @brief 启动机器人控制器，完成初始化后获取timestamp，启动计时器
 *
 * @return int 返回初始化状态，0为成功
 */
int RobotController::Run() {

  if (robot_initialized_ != 1) {
    return -1;
  }
  // 初始化当前关节状态
  auto current = getJointState();
  {
    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    current_joint_state_ = current;
    last_joint_state_ = current;
  }
  // 初始化目标关节状态
  last_joint_state_ = current;
  timer_->start();
  // log print
  std::cout << "Controller start :" << name_ << std::endl;
  return 0;
}
int RobotController::Stop() {
  if (timer_) {
    timer_->stop();
  }
  return 0;
}
RobotController::~RobotController() { Stop(); }
/**
 * @brief 设置目标关节状态
 *
 * @param target_joint_state
 * @return int
 */
int RobotController::setTarget(RobotJointState target_joint_state) {
  std::unique_lock<std::mutex> lock(joint_state_mutex_);
  target_joint_state_ = target_joint_state;
  return 0;
}
RobotController::RobotJointState RobotController::getTarget() {
  std::unique_lock<std::mutex> lock(joint_state_mutex_);
  auto target = target_joint_state_;
  return target;
}