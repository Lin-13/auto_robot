#include "robot_interface/robot.h"
#include <iostream>
#include <utils/matrix_utils.h> // interplotePose
Robot::Robot(std::shared_ptr<RobotController> robot_controller,
             std::shared_ptr<RobotTopology> robot_topology)
    : controller_(robot_controller), topology_(robot_topology) {}
/**
 * @brief 初始化并启动控制器，只有start后TrajectoryMove等才能生效
 *
 * @param timer_period 控制器定时器周期，单位为毫秒
 * @return int 返回启动状态，0为成功
 */
int Robot::start(std::chrono::milliseconds timer_period) {
  enable = 0;
  if (controller_ == nullptr) {
    return -1;
  }
  if (controller_->Initialize(timer_period) != 0) {
    return -1;
  }
  if (controller_->Run() != 0) {
    return -1;
  }
  enable = 1;
  return 0;
}
/**
 * @brief 停止控制器，TrajectoryMove等失效
 *
 * @return int 返回停止状态，0为成功
 */
int Robot::stop() {
  enable = 0;
  for (auto &[name, timer] : timers_) {
    timer->stop();
  }
  if (controller_ != nullptr) {
    controller_->Stop();
  }
  return 0;
}
/**
 * @brief 轨迹绝对位置移动，只能在start后调用
 *
 * @param trajectory 轨迹，时间-位置（joint-Eigen::VectorXd）
 * @param interval 移动间隔，单位为毫秒  `默认100ms`
 * @param log 是否打印日志  `默认0`
 * @param start_now 是否立即开始移动，0为否，1为是  `默认1`
 * @return int 返回移动状态，0为成功，-1为失败
 */
int Robot::MoveJoint(const Trajectory &trajectory,
                     std::chrono::milliseconds interval, int log,
                     int start_now) {
  if (enable == 0) {
    return -1;
  }
  Trajectory traj = trajectory;
  if (checkTrajectoryType(traj) != "joint") {
    return -1;
  }
  std::ranges::sort(
      traj, [](const auto &a, const auto &b) { return a.first < b.first; });
  // 启动计时器
  std::function<int(void)> time_cb =
      [this, traj, log, cnt = 0,
       start_time = std::chrono::steady_clock::now()]() mutable {
        static double eps = 0.05;
        if (cnt == 0) {
          start_time = std::chrono::steady_clock::now();
        }
        cnt++;
        auto time = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::microseconds>(
                       time - start_time)
                       .count() /
                   1.0e6;
        if (t > traj.back().first + eps) {
          if (log) {
            std::cout << "MoveJoint: Stop" << std::endl;
          }
          return -1; // Stop Traj Timer
        }
        Eigen::MatrixXd pos = this->interpolate(t, traj);
        if (log) {
          std::cout << "MoveJoint: t = " << t << ", pos = " << pos.transpose()
                    << std::endl;
        }
        this->controller_->setTarget(RobotController::RobotJointState(pos));
        return 0;
      };
  Timer::Ptr timer = Timer::create("MoveJoint", time_cb, interval, nullptr);
  timers_.insert_or_assign("DefaultTimer", timer);
  // C++17, 若键不存在直接构造并插入，若存在则更新值
  if (start_now) {
    timer->start();
  }

  return 0;
}
/**
 * @brief 轨迹相对位置移动，只能在start后调用
 *
 * @param trajectory 轨迹，时间-位置[joint-Eigen::VectorXd）
 * @param interval 移动间隔，单位为毫秒  `默认100ms`
 * @param log 是否打印日志  `默认0`
 * @param start_now 是否立即开始移动，0为否，1为是  `默认1`
 * @return int 返回移动状态，0为成功，-1为失败
 */
int Robot::MoveJointRelative(const Trajectory &trajectory,
                             std::chrono::milliseconds interval, int log,
                             int start_now) {
  if (enable == 0) {
    return -1;
  }
  Trajectory traj = trajectory;
  // 检查轨迹，分析轨迹类型
  if (checkTrajectoryType(traj) != "joint") {
    return -1;
  }
  std::ranges::sort(
      traj, [](const auto &a, const auto &b) { return a.first < b.first; });
  Eigen::VectorXd joint_pos = controller_->getJointState().joint_state;
  if (joint_pos.rows() != traj.front().second.rows()) {
    return -1;
  }
  // 计算相对位置
  for (auto &pair : traj) {
    pair.second += joint_pos;
  }
  return MoveJoint(trajectory, interval, log, start_now);
}
/**
 * @brief 轨迹绝对位置移动，只能在start后调用
 *
 * @param trajectory 轨迹，位置[pose-Matrix4d）
 * @param interval 移动间隔，单位为毫秒  `默认100ms`
 * @param log 是否打印日志  `默认0`
 * @param start_now 是否立即开始移动，0为否，1为是  `默认1`
 * @return int 返回移动状态，0为成功，-1为失败
 */
int Robot::MovePose(const Trajectory &trajectory,
                    std::chrono::milliseconds interval, int log,
                    int start_now) {
  if (enable == 0) {
    return -1;
  }
  Trajectory traj = trajectory;
  // 检查轨迹，分析轨迹类型
  if (checkTrajectoryType(traj) != "pose") {
    return -1;
  }
  if (topology_ == nullptr) {
    if (log) {
      std::cout << "Robot::MovePose: topology_ is null" << std::endl;
    }
    return -1;
  }
  // sort
  std::ranges::sort(
      traj, [](const auto &a, const auto &b) { return a.first < b.first; });
  // 启动计时器
  std::function<int(void)> pose_time_cb =
      [this, traj, log, cnt = 0,
       start_time = std::chrono::steady_clock::now()]() mutable {
        static double eps = 0.05;
        static Eigen::VectorXd last_joint =
            controller_->getJointState().joint_state;
        if (cnt == 0) {
          start_time = std::chrono::steady_clock::now();
        }
        cnt++;
        auto start = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::microseconds>(
                       start - start_time)
                       .count() /
                   1.0e6;
        Eigen::Matrix4d pos = this->interpolatePose(t, traj);
        int ret;
        Eigen::VectorXd pos_joint = topology_->trans_inv(pos, last_joint, &ret);
        if (ret != 0) { // trans_inv E_NOERROR=0
          if (log) {
            std::cout << "MovePose: t = " << t << ", trans_inv:[\n"
                      << pos << "]\nfailed, ret = " << ret << std::endl;
          }
          return -1; // stop traj timer
        }
        last_joint = pos_joint;
        this->controller_->setTarget(
            RobotController::RobotJointState(pos_joint));
        // 确保setTarget至少会调用一次（当traj只有t=0时）
        if (t > traj.back().first + eps) {
          if (log) {
            std::cout << "MovePose: Stop" << std::endl;
          }
          return -1; // Stop Traj Timer
        }
        auto end = std::chrono::steady_clock::now();
        if (log) {
          std::cout << "MovePose: t = "
                    << std::chrono::duration_cast<std::chrono::microseconds>(
                           end - start_time)
                               .count() /
                           1.0e6
                    << ", solve time = "
                    << std::chrono::duration_cast<std::chrono::microseconds>(
                           end - start)
                               .count() /
                           1.0e6
                    << "\njoint=" << pos_joint.transpose() << "\npos = \n"
                    << pos << std::endl;
        }
        return 0;
      };
  Timer::Ptr timer = Timer::create("MovePose", pose_time_cb, interval, nullptr);
  timers_.insert_or_assign("DefaultTimer", timer);
  // C++17, 若键不存在直接构造并插入，若存在则更新
  if (start_now) {
    timer->start();
  }
  return 0;
}
/**
 * @brief 轨迹相对位置移动，只能在start后调用,以基坐标系进行相对运动
 *
 * @param trajectory 轨迹，位置[pose-Matrix4d）
 * @param interval 移动间隔，单位为毫秒  `默认100ms`
 * @param log 是否打印日志  `默认0`
 * @param start_now 是否立即开始移动，0为否，1为是  `默认1`
 * @return int 返回移动状态，0为成功，-1为失败
 */
int Robot::MovePoseRelative(const Trajectory &trajectory,
                            std::chrono::milliseconds interval, int log,
                            int start_now) {
  if (enable == 0) {
    return -1;
  }
  Trajectory traj = trajectory;
  // 检查轨迹，分析轨迹类型
  if (checkTrajectoryType(traj) != "pose") {
    return -1;
  }
  if (topology_ == nullptr) {
    if (log) {
      std::cout << "Robot::MovePoseRelative: topology_ is null" << std::endl;
    }
    return -1;
  }
  // sort
  std::ranges::sort(
      traj, [](const auto &a, const auto &b) { return a.first < b.first; });
  Eigen::VectorXd pos = controller_->getJointState().joint_state;
  // std::cout << pos.rows() << std::endl;
  // std::cout << pos.transpose() << std::endl;
  int ret = 0;
  if (pos.rows() != controller_->getJointNum()) {
    if (log) {
      std::cout
          << "MovePoseRelative: pos rows not equal to joint num, pos rows = "
          << pos.rows()
          << ", controller  joint num = " << controller_->getJointNum()
          << std::endl;
    }
    return -1;
  }
  Eigen::Matrix4d T = topology_->trans(pos, &ret);
  if (ret != 0) { // trans E_NOERROR=0
    if (log) {
      std::cout << "MovePoseRelative: trans failed, ret = " << ret << "T = \n"
                << T << std::endl;
    }
    return -1;
  }
  // std::cout << "MovePoseRelative:Current joints: " << pos.transpose() << "\n"
  //           << "MovePoseRelative: T = \n"
  //           << T << std::endl;
  for (auto &pair : traj) {
    pair.second = pair.second * T;
    // pair.second = T * pair.second;
    // TODO: 在末端坐标系下进行偏移在插值的时候会出现问题会出问题
    if (log) {
      std::cout << "MovePoseRelative: t = " << pair.first << "\npos (deg)= \n"
                << topology_->trans_inv(pair.second, pos).transpose() * 180 / pi
                << "\nT = \n"
                << pair.second << std::endl;
    }
  }
  return MovePose(traj, interval, log, start_now);
}
/**
 * @brief 检查轨迹是否合法
 *
 * @param trajectory 轨迹
 * @return std::string 检查结果 返回joint , pose , other
 */
std::string Robot::checkTrajectoryType(const Trajectory &trajectory) {
  if (trajectory.size() == 0) {
    return "other";
  }
  int rows = trajectory[0].second.rows();
  int cols = trajectory[0].second.cols();
  std::string type = "other";
  if (rows == 4 && cols == 4) {
    type = "pose";
  } else if (rows == controller_->getJointNum() && cols == 1) {
    type = "joint";
  } else {
    return "other";
  }
  if (std::ranges::any_of(trajectory, [rows, cols](const auto &pair) {
        return pair.first < 0 || pair.second.cols() != cols ||
               pair.second.rows() != rows;
      })) {
    type = "other";
  }
  return type;
}
/**
 * @brief
 * 插值函数，当时间t小于0时，返回0矩阵，
 * 当时间t小于轨迹第一个时间点时，返回轨迹第一个点，
 * 当时间t大于轨迹最后一个时间点时，返回轨迹最后一个点，
 * 当时间t在轨迹区间内时，进行线性插值。
 *
 * @param t 时间
 * @param trajectory 轨迹
 * @return Eigen::MatrixXd 插值结果
 */
Eigen::MatrixXd Robot::interpolate(const double t,
                                   const Trajectory &trajectory) {
  if (checkTrajectoryType(trajectory) != "joint") {
    return Eigen::MatrixXd::Zero(getJointNum(), 1);
  }
  if (t < 0) {
    return Eigen::MatrixXd::Zero(getJointNum(), 1);
  }
  int rows = trajectory[0].second.rows();
  int cols = trajectory[0].second.cols();
  // 溢出
  if (t <= trajectory.front().first) {
    return trajectory.front().second;
  }
  if (t >= trajectory.back().first) {
    return trajectory.back().second;
  }
  // 插值
  for (int i = 0; i < trajectory.size() - 1; i++) {
    if (t >= trajectory[i].first && t <= trajectory[i + 1].first) {
      double t0 = trajectory[i].first;
      double t1 = trajectory[i + 1].first;
      Eigen::MatrixXd p0 = trajectory[i].second;
      Eigen::MatrixXd p1 = trajectory[i + 1].second;
      return p0 + (p1 - p0) * (t - t0) / (t1 - t0);
    }
  }
  return Eigen::MatrixXd::Zero(rows, cols);
}
/**
 * @brief 矩阵插值函数
 *
 * @param t 时间 t>=0
 * @param trajectory 轨迹
 * @return Eigen::Matrix4d 插值结果
 */
Eigen::Matrix4d Robot::interpolatePose(const double t,
                                       const Trajectory &trajectory) {
  if (checkTrajectoryType(trajectory) != "pose") {
    return Eigen::Matrix4d::Identity();
  }
  int rows = trajectory[0].second.rows();
  int cols = trajectory[0].second.cols();
  if (t < 0) {
    return trajectory.front().second;
  }
  // 溢出
  if (t <= trajectory.front().first) {
    return trajectory.front().second;
  }
  if (t >= trajectory.back().first) {
    return trajectory.back().second;
  }
  // 插值
  for (int i = 0; i < trajectory.size() - 1; i++) {
    if (t >= trajectory[i].first && t <= trajectory[i + 1].first) {
      double t0 = trajectory[i].first;
      double t1 = trajectory[i + 1].first;
      double lambda = (t - t0) / (t1 - t0); // 0<=lambda <=1
      Eigen::Matrix4d T0 = trajectory[i].second;
      Eigen::Matrix4d T1 = trajectory[i + 1].second;
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      if (interpolate_se3 != 1) {
        Eigen::Matrix3d R0 = T0.block<3, 3>(0, 0);
        Eigen::Matrix3d R1 = T1.block<3, 3>(0, 0);
        Eigen::Vector3d p0 = T0.block<3, 1>(0, 3);
        Eigen::Vector3d p1 = T1.block<3, 1>(0, 3);
        Eigen::Matrix3d deltaR = R1 * R0.transpose();
        Eigen::Matrix3d dR = so3ToSO3(lambda * SO3Toso3(deltaR));
        T.block<3, 3>(0, 0) = dR * R0;

        // Eigen::Quaterniond q0(R0), q1(R1);
        // Eigen::Quaterniond q_interp = q0.slerp(lambda, q1);
        // T.block<3, 3>(0, 0) = q_interp.toRotationMatrix(); //四元数插值
        T.block<3, 1>(0, 3) = p0 + (p1 - p0) * lambda;
      } else {
        Eigen::Matrix4d deltaT = T1 * T0.inverse();
        Eigen::Matrix4d dT = se3ToSE3(lambda * SE3Tose3(deltaT));
        T = dT * T0;
      }
      return T;
    }
  }
  return trajectory.back().second;
}
/**
 * @brief 获取机器人关节数量
 *
 * @return int 关节数量
 */
int Robot::getJointNum() {
  if (controller_ == nullptr || topology_ == nullptr) {
    throw std::runtime_error(
        "Robot::getJointNum: controller or topology is null");
  }
  if (controller_->getJointNum() != topology_->getJointNum()) {
    throw std::runtime_error(
        "Robot::getJointNum: controller joint num not equal to topology joint "
        "num");
  }
  return controller_->getJointNum();
}
/**
 * @brief 获取当前位姿
 *
 * @return Eigen::Matrix4d 当前位姿
 */
Eigen::Matrix4d Robot::currentPose() {
  if (controller_ == nullptr || topology_ == nullptr) {
    throw std::runtime_error(
        "Robot::currentPose: controller or topology is null");
  }
  RobotController::RobotJointState joint_state = controller_->getJointState();
  return topology_->trans(joint_state.joint_state);
}
/**
 * @brief 获取当前关节状态
 *
 * @return Eigen::VectorXd 当前关节状态
 */
Eigen::VectorXd Robot::currentJointState() {
  if (controller_ == nullptr) {
    throw std::runtime_error(
        "Robot::currentJointState: controller or topology is null");
  }
  RobotController::RobotJointState joint_state = controller_->getJointState();
  return joint_state.joint_state;
}