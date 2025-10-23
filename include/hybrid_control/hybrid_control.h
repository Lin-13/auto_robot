#ifndef HYBRID_CONTROL_H_
#define HYBRID_CONTROL_H_
#include "ft_sensor/force_control.h"
#include "ft_sensor/ft_calib.h"
#include "ft_sensor/ft_sensor.h"
#include "robot_interface/robot.h"
#include "utils/matrix_utils.h"
#include "utils/transform_tree.h"
#include <chrono>
#include <fmt/format.h>
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
                 std::shared_ptr<TransformTree> transform_tree) {
    robot_ = robot;
    transform_tree_ = transform_tree;
    force_sensor_ = force_sensor;
    force_sensor->bindPoseDetector([robot]() { return robot->currentPose(); });
    admittance_controller_ = std::make_shared<ControllerType>(5, 80, 1000);
    admittance_controller_->setDesiredPos(0);
    admittance_controller_->setDesiredForce(0);
    admittance_controller_->setForceSensor([force_sensor, robot]() -> double {
      Eigen::Vector<double, 6> wrench =
          force_sensor->getCompensatedWrench().block<6, 1>(0, 0);
      Eigen::Vector3d force = wrench.block<3, 1>(0, 0);
      force = robot->currentPose().block<3, 3>(0, 0) * force; // 机器人基坐标系
      return force.z();                                       // z
    });
    auto initial_pos = robot_->currentPose();
    admittance_controller_->setPositionSensor([robot, initial_pos]() -> double {
      return robot->currentPose()(2, 3) - initial_pos(2, 3); // z
    });
    admittance_controller_->setPositionUpdate(
        [robot, initial_pos](double pos) -> int {
          auto new_pose = initial_pos;
          new_pose(2, 3) = pos + initial_pos(2, 3);
          robot->MoveJointToPose(new_pose);
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

private:
  std::shared_ptr<Robot> robot_;
  std::shared_ptr<FTSensorGravityCompensation> force_sensor_;
  std::shared_ptr<TransformTree> transform_tree_;
  std::shared_ptr<ControllerType> admittance_controller_;
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
    robot_ = robot;
    transform_tree_ = transform_tree;
    force_sensor_ = force_sensor;
    force_sensor->bindPoseDetector([robot]() { return robot->currentPose(); });
    // m,kv,k
    auto I3 = Eigen::Matrix3d::Identity();
    admittance_controller_ =
        std::make_shared<ControllerType>(10 * I3, 100 * I3, 300 * I3);
    admittance_controller_->setDesiredPos(Eigen::Vector3d::Zero());
    admittance_controller_->setDesiredForce(Eigen::Vector3d::Zero());
    admittance_controller_->setForceSensor([force_sensor,
                                            robot]() -> Eigen::Vector3d {
      Eigen::Vector<double, 6> wrench =
          force_sensor->getCompensatedWrench().block<6, 1>(0, 0);
      Eigen::Vector3d force = wrench.block<3, 1>(0, 0);
      force = robot->currentPose().block<3, 3>(0, 0) * force; // 机器人基坐标系
      return force;
    });
    auto initial_pos = robot_->currentPose();
    admittance_controller_->setPositionSensor(
        [robot, initial_pos]() -> Eigen::Vector3d {
          return robot->currentPose().block<3, 1>(0, 3) -
                 initial_pos.block<3, 1>(0, 3); // z
        });
    admittance_controller_->setPositionUpdate(
        [robot, initial_pos](Eigen::Vector3d pos) -> int {
          auto new_pose = initial_pos;
          new_pose.block<3, 1>(0, 3) = pos + initial_pos.block<3, 1>(0, 3);
          robot->MoveJointToPose(new_pose);
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
  virtual ~BaseController3d() = default;

private:
  std::shared_ptr<Robot> robot_;
  std::shared_ptr<FTSensorGravityCompensation> force_sensor_;
  std::shared_ptr<TransformTree> transform_tree_;
  std::shared_ptr<ControllerType> admittance_controller_;
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