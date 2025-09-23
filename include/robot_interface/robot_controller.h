#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <robot_interface/timer.h>
/**
 * @brief PID控制器配置参数
 *
 */
struct ControllerConfig {
  double k_p = 1;
  double k_v = 0;
};
/**
 * @brief 机器人控制器基类——在关节空间中控制机器人位置
 *        该类存在状态：
 *            initialized_
 *            target_joint_state_
 *        子类必须在Initialize()函数中执行初始化机器人状态并创建定时器
 *        子类必须实现getJointState和setJointState函数，并且确保无阻塞，
 *        该函数会在定时器触发时被调用
 *
 *
 */
class RobotController {
public:
  friend class Robot;
  using Ptr = std::shared_ptr<RobotController>;
  /**
   * @brief 关节状态结构体
   * TODO: 加入关节速度的控制[joints*2,1]
   *
   */
  struct RobotJointState {
    RobotJointState() = default;
    RobotJointState(Eigen::VectorXd joint_state)
        : joint_state(joint_state), joints(joint_state.rows()) {}
    Eigen::VectorXd joint_state; // [joints,1]
    int joints = 6;
    std::chrono::time_point<std::chrono::steady_clock> timestamp =
        std::chrono::steady_clock::now();
  };
  struct RobotCartesianState {
    int joints = 6;
    Eigen::Matrix4d trans;
    Eigen::VectorXd v;
    Eigen::VectorXd omega;
    std::chrono::time_point<std::chrono::steady_clock> timestamp =
        std::chrono::steady_clock::now();
  };
  RobotController(std::string name, int num_joints = 6,
                  ControllerConfig config = ControllerConfig());
  int getJointNum() { return num_joints_; }
  virtual int Initialize(std::chrono::milliseconds timer_period = 33ms) = 0;
  virtual int Run();
  virtual int Stop();
  virtual int setTarget(RobotJointState target_joint_state);
  virtual RobotJointState getTarget();
  virtual ~RobotController();

protected:
  /**
   * @brief 调用getJointState和setJointState实现实时控制
   *  TODO: [pose,vel]状态空间的控制其实现
   *
   * @return int
   */
  virtual int timer_cb();

  /**
   * @brief 获取当前状态:pose
   *  TODO: [pose,vel]的控制其实现
   *
   * @return Eigen::VectorXd
   */
  virtual RobotJointState getJointState() = 0;

protected:
  /**
   * @brief 写入关节状态
   *
   * @param q 关节状态
   * @return int 0:success -1:fail
   */
  virtual int setJointState(RobotJointState q) = 0;
  // 机器人配置
  Timer::Ptr timer_;
  std::string name_;
  int num_joints_;
  int robot_initialized_ = 0;
  // 目标关节状态和控制其状态，用于机器人控制
  ControllerConfig config_;
  RobotJointState target_joint_state_, last_target_joint_state_,
      current_joint_state_, last_joint_state_;
  std::mutex joint_state_mutex_;
  // time
  uint64_t start_timestamp_ = 0, current_timestamp_ = 0;
  uint64_t robot_start_timestamp_ = 0, robot_current_timestamp_ = 0;
};
