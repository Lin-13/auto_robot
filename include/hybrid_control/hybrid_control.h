#ifndef HYBRID_CONTROL_H_
#define HYBRID_CONTROL_H_
#include "ft_sensor/force_control.h"
#include "ft_sensor/ft_calib.h"
#include "ft_sensor/ft_sensor.h"
#include "robot_interface/robot.h"
#include "utils/debug_utils.h"
#include "utils/matrix_utils.h"
#include "utils/transform_tree.h"
#include <chrono>
#include <fmt/format.h>
#include <fstream>
#include <utils/signals_utils.h>
/**
 * @brief 简单的机器人力位混合控制
 *
 */
class BaseController {
  using ControllerType = AdmittanceController1d;

public:
  /**
   * @brief 接受已经完成初始化的robot, force_sensor, transform_tree 实例
   *
   * @param robot
   * @param transform_tree
   * @param force_sensor
   */
  BaseController(std::shared_ptr<Robot> robot,
                 std::shared_ptr<FTSensorGravityCompensation> force_sensor,
                 std::shared_ptr<TransformTree> transform_tree, int axis = 2) {
    log_file_.open("pose_log.txt", std::ios::out | std::ios::trunc);
    robot_ = robot;
    transform_tree_ = transform_tree;
    force_sensor_ = force_sensor;
    axis_ = axis;
    force_sensor->bindPoseDetector([robot]() { return robot->currentPose(); });
    admittance_controller_ = std::make_shared<ControllerType>(200, 2000, 200);
    admittance_controller_->setDesiredPos(0);
    admittance_controller_->setDesiredForce(0);
    admittance_controller_->setForceSensor([force_sensor, robot, axis,
                                            this]() -> double {
      static auto start = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      double t =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
              .count() /
          1000.0;
      Eigen::Vector<double, 6> wrench =
          force_sensor->getCompensatedWrench().block<6, 1>(0, 0);
      Eigen::Vector3d force = wrench.block<3, 1>(0, 0);
      force(0) = 0;
      force(1) = 0; //只取z轴，x,y度数不准
      force = robot->currentPose().block<3, 3>(0, 0) * force; // 机器人基坐标系
      log_file_ << "t : " << t << " force : " << force.transpose() << std::endl;
      return force[axis]; // z
    });
    auto initial_pos = robot_->currentPose();
    ref_position_ = initial_pos(axis, 3);
    // throw std::runtime_error("需要按照BaseController3d重写该部分代码");
    // admittance_controller_->setPositionSensor([robot, axis, this]() -> double
    // {
    //   return robot->currentPose()(axis, 3) - ref_position_;
    // });
    admittance_controller_->setVirtualPositionSensor(0.0);
    admittance_controller_->setPositionUpdate([robot, axis, initial_pos,
                                               this](double pos) -> int {
      auto new_pose = initial_pos;
      new_pose(axis, 3) = pos + ref_position_;
      std::cout << "Target " << axis << " : " << std::setprecision(4) << pos
                << std::endl;
      static auto start = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      double t =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
              .count() /
          1000.0;
      log_file_ << "t : " << t
                << " xyz : " << Eigen::Vector3d(pos, 0, 0).transpose()
                << std::endl;
      Eigen::VectorXd joint =
          robot->topology()->trans_inv(new_pose, robot->currentJointState());
      log_file_ << "t : " << t << " joint : " << joint.transpose() << std::endl;
      log_file_
          << "t : " << t << " actual_xyz : "
          << (robot->currentPose() - initial_pos).block<3, 1>(0, 3).transpose()
          << std::endl;
      log_file_ << "t : " << t
                << " actual_joint : " << robot->currentJointState().transpose()
                << std::endl;
      // robot->MoveJointToPose(new_pose);
      // MoveJointToPose internally calls setTarget
      robot->controller()->setTarget(RobotController::RobotJointState(joint));
      return 0;
    });
  }
  std::shared_ptr<ControllerType> getAdmittanceController() {
    return admittance_controller_;
  }
  int updateOnce() {
    admittance_controller_->updateOnce();
    return 0;
  }
  virtual ~BaseController() = default;
  double getPos() { return admittance_controller_->getPos(); }
  double getForce() { return admittance_controller_->getForce(); }
  double getDesiredPos() { return admittance_controller_->getDesiredPos(); }
  double getDesiredForce() { return admittance_controller_->getDesiredForce(); }
  void setDesiredPos(double pos) { admittance_controller_->setDesiredPos(pos); }
  void setDesiredForce(double force) {
    admittance_controller_->setDesiredForce(force);
  }

private:
  std::shared_ptr<Robot> robot_;
  std::shared_ptr<FTSensorGravityCompensation> force_sensor_;
  std::shared_ptr<TransformTree> transform_tree_;
  std::shared_ptr<ControllerType> admittance_controller_;
  int axis_; //轴
  double ref_position_;
  std::mutex ref_mutex_;
  std::fstream log_file_;
};
class BaseController3d {
  using ControllerType = AdmittanceController3d;

public:
  /**
   * @brief 接受已经完成初始化的robot, force_sensor, transform_tree 实例
   *
   * @param robot
   * @param transform_tree
   * @param force_sensor
   */
  BaseController3d(std::shared_ptr<Robot> robot,
                   std::shared_ptr<FTSensorGravityCompensation> force_sensor,
                   std::shared_ptr<TransformTree> transform_tree) {
    log_file_.open("pose_log.txt", std::ios::out | std::ios::trunc);
    robot_ = robot;
    transform_tree_ = transform_tree;
    force_sensor_ = force_sensor;
    // force_sensor->bindPoseDetector([robot]() { return robot->currentPose();
    // });
    auto I3 = Eigen::Matrix3d::Identity();
    // force sensor filter

    // config controller
    // m,kv,K
    admittance_controller_ =
        std::make_shared<ControllerType>(7000 * I3, 5000 * I3, 200 * I3);
    admittance_controller_->setDesiredPos(Eigen::Vector3d::Zero());
    admittance_controller_->setDesiredForce(Eigen::Vector3d::Zero());
    admittance_controller_->setForceSensor([this, force_sensor,
                                            robot]() -> Eigen::Vector3d {
      static auto start = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      double t =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
              .count() /
          1000.0;
      Eigen::Vector<double, 6> wrench =
          force_sensor->getCompensatedWrench().block<6, 1>(0, 0);
      Eigen::Vector3d force = wrench.block<3, 1>(0, 0);
      force = robot->currentPose().block<3, 3>(0, 0) * force; // 机器人基坐标系
      log_file_ << "t : " << t << " force : " << force.transpose() << std::endl;
      return force;
    });
    // 初始化虚拟传感器
    admittance_controller_->setVirtualPositionSensor(Eigen::Vector3d::Zero());
    ref_position_ = robot_->currentPose();
    // admittance_controller_->setPositionSensor(
    //     [robot, initial_pos]() -> Eigen::Vector3d {
    //       // return robot->currentPose().block<3, 1>(0, 3) -
    //       //        initial_pos.block<3, 1>(0, 3);
    //     });
    admittance_controller_->setPositionUpdate([robot, this](
                                                  Eigen::Vector3d pos) -> int {
      auto new_pose = ref_position_;
      static auto start = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      double t =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
              .count() /
          1000.0;
      new_pose.block<3, 1>(0, 3) = pos + ref_position_.block<3, 1>(0, 3);
      log_file_ << "t : " << t << " xyz : " << pos.transpose() << std::endl;
      Eigen::VectorXd joint =
          robot->topology()->trans_inv(new_pose, robot->currentJointState());
      log_file_ << "t : " << t << " joint : " << joint.transpose() << std::endl;
      log_file_ << "t : " << t << " actual_xyz : "
                << (robot->currentPose() - ref_position_)
                       .block<3, 1>(0, 3)
                       .transpose()
                << std::endl;
      log_file_ << "t : " << t
                << " actual_joint : " << robot->currentJointState().transpose()
                << std::endl;
      // robot->MoveJointToPose(new_pose);
      // MoveJointToPose internally calls setTarget
      robot->controller()->setTarget(RobotController::RobotJointState(joint));
      return 0;
    });
  }
  Eigen::Vector3d getPos() { return admittance_controller_->getPos(); }
  Eigen::Vector3d getForce() { return admittance_controller_->getForce(); }
  Eigen::Vector3d getDesiredPos() {
    return admittance_controller_->getDesiredPos();
  }
  Eigen::Vector3d getDesiredForce() {
    return admittance_controller_->getDesiredForce();
  }
  void setDesiredPos(const Eigen::Vector3d &pos) {
    admittance_controller_->setDesiredPos(pos);
  }
  void setDesiredForce(const Eigen::Vector3d &force) {
    admittance_controller_->setDesiredForce(force);
  }
  void MoveDesiredPositionRel(const Eigen::Vector3d &pos) {
    Eigen::Vector3d current_des = admittance_controller_->getDesiredPos();
    admittance_controller_->setDesiredPos(current_des + pos);
  }
  void MoveDesiredForceRel(const Eigen::Vector3d &force) {
    Eigen::Vector3d current_des = admittance_controller_->getDesiredForce();
    admittance_controller_->setDesiredForce(current_des + force);
  }
  std::shared_ptr<ControllerType> getAdmittanceController() {
    return admittance_controller_;
  }
  int updateOnce() {
    admittance_controller_->updateOnce();
    return 0;
  }
  virtual ~BaseController3d() {
    log_file_.close();
    std::cerr << "log file saved.";
  }

private:
  std::shared_ptr<Robot> robot_;
  std::shared_ptr<FTSensorGravityCompensation> force_sensor_;
  std::shared_ptr<TransformTree> transform_tree_;
  std::shared_ptr<ControllerType> admittance_controller_;
  std::shared_ptr<DownSampleFilter> downsample_filter_;
  std::fstream log_file_;
  Eigen::Matrix4d ref_position_;
  std::mutex ref_mutex_;
};
/**
 * @brief 双机器人力位混合控制
 *
 */
class HybridController {
public:
  HybridController(std::shared_ptr<Robot> robot_left,
                   std::shared_ptr<Robot> robot_right,
                   std::shared_ptr<TransformTree> transform_tree);
  ~HybridController();

private:
  // 需要注入的依赖
  std::shared_ptr<TransformTree> transform_tree_;
  std::shared_ptr<Robot> robot_left_;
  std::shared_ptr<Robot> robot_right_;
};
#endif // HYBRID_CONTROL_H_