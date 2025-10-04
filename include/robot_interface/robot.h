#pragma once
#include "robot_interface/robot_controller.h"
#include "robot_interface/robot_topology.h"
#include "robot_interface/timer.h"
#include <cmath>
/**
 * @brief 结合RobotController和RobotTopology实现高级控制
 * TODO:
 * MovePose需要进行复杂的旋转矩阵处理，是否需要在该类进行还是在更高级的代码中实现？
 */
class Robot {
public:
  // 轨迹，时间-位置（Matrix4d or Eigen::VectorXd）
  using Trajectory = std::vector<std::pair<double, Eigen::MatrixXd>>;
  Robot(RobotController::Ptr controller, RobotTopology::Ptr topology);
  int start(std::chrono::milliseconds timer_period);
  int stop();
  ~Robot() = default;
  int getJointNum();
  int MoveJoint(const Trajectory &trajectory,
                std::chrono::milliseconds interval = 100ms, int log = 0,
                int start_now = 1);
  int MovePose(const Trajectory &trajectory,
               std::chrono::milliseconds interval = 100ms, int log = 0,
               int start_now = 1);
  int MoveJointRelative(const Trajectory &trajectory,
                        std::chrono::milliseconds interval = 100ms, int log = 0,
                        int start_now = 1);
  int MovePoseRelative(const Trajectory &trajectory,
                       std::chrono::milliseconds interval = 100ms, int log = 0,
                       int start_now = 1);
  std::string checkTrajectoryType(const Trajectory &trajectory);
  RobotTopology::Ptr topology() const { return topology_; }
  RobotController::Ptr controller() const { return controller_; }
  Eigen::Matrix4d currentPose();
  Eigen::VectorXd currentJointState();

  // 显式启动计时器，在多机器人同步时使用
  /**
   * @brief 启动计时器
   * @param timer_name 计时器名称
   * @return int 0:success -1:fail
   */
  int startTimer(const std::string timer_name = "DefaultTimer") {
    if (timers_.count(timer_name)) {
      timers_[timer_name]->start();
      return 0;
    }
    return -1;
  }
  int stopTimer(const std::string timer_name = "DefaultTimer") {
    if (timers_.count(timer_name)) {
      timers_[timer_name]->stop();
      return 0;
    }
    return -1;
  }
  bool isTimerRunning(const std::string timer_name = "DefaultTimer") {
    if (timers_.count(timer_name)) {
      return timers_[timer_name]->state() == Timer::State::Running;
    }
    return false;
  }
  /**
   * @brief 使能SE3插值
   * @param enable 0:disable 1:enable
   */
  void interpolatePoseInSE3(bool enable = 1) { interpolate_se3 = enable; }

private:
  int enable = 0; //使能标志位
  int interpolate_se3 = 0;
  Eigen::MatrixXd interpolate(const double t, const Trajectory &trajectory);
  Eigen::Matrix4d interpolatePose(const double t, const Trajectory &trajectory);
  std::unordered_map<std::string, Timer::Ptr> timers_; //多个Timers
  //   std::unordered_map<std::string, Trajectory> trajectories_;
  std::mutex trajectory_mutex_;
  RobotController::Ptr controller_;
  RobotTopology::Ptr topology_;
};
