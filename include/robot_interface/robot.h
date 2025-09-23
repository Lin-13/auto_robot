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
                std::chrono::milliseconds interval = 100ms, int log = 0);
  int MovePose(const Trajectory &trajectory,
               std::chrono::milliseconds interval = 100ms, int log = 0);
  int MoveJointRelative(const Trajectory &trajectory,
                        std::chrono::milliseconds interval = 100ms,
                        int log = 0);
  int MovePoseRelative(const Trajectory &trajectory,
                       std::chrono::milliseconds interval = 100ms, int log = 0);
  std::string checkTrajectoryType(const Trajectory &trajectory);
  RobotTopology::Ptr topology() const { return topology_; }
  RobotController::Ptr controller() const { return controller_; }
  Eigen::Matrix4d currentPose();
  Eigen::VectorXd currentJointState();

private:
  int enable = 0; //使能标志位
  Eigen::MatrixXd interplote(const double t, const Trajectory &trajectory);
  Eigen::Matrix4d interplotePose(const double t, const Trajectory &trajectory);
  std::unordered_map<std::string, Timer::Ptr> timers_; //多个Timers
  //   std::unordered_map<std::string, Trajectory> trajectories_;
  std::mutex trajectory_mutex_;
  RobotController::Ptr controller_;
  RobotTopology::Ptr topology_;
};