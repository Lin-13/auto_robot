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
    robot_ = robot;
    transform_tree_ = transform_tree;
    force_sensor_ = force_sensor;
    axis_ = axis;
    force_sensor->bindPoseDetector([robot]() { return robot->currentPose(); });
    // sensor filter
    // std::shared_ptr<DownSampleFilter> downsample_filter_ =
    //     std::make_shared<DownSampleFilter>(30, 1000, 100, 4);
    // downsample_filter_->setPusher([force_sensor, axis]() -> Eigen::Vector3d {
    //   return force_sensor->getCompensatedWrench().block<3, 1>(0, 0);
    // });
    admittance_controller_ = std::make_shared<ControllerType>(5, 80, 1000);
    admittance_controller_->setDesiredPos(0);
    admittance_controller_->setDesiredForce(0);
    admittance_controller_->setForceSensor([force_sensor, robot,
                                            axis]() -> double {
      Eigen::Vector<double, 6> wrench =
          force_sensor->getCompensatedWrench().block<6, 1>(0, 0);
      Eigen::Vector3d force = wrench.block<3, 1>(0, 0);
      force = robot->currentPose().block<3, 3>(0, 0) * force; // 机器人基坐标系
      return force[axis];                                     // z
    });
    auto initial_pos = robot_->currentPose();
    admittance_controller_->setPositionSensor(
        [robot, initial_pos, axis]() -> double {
          return robot->currentPose()(axis, 3) - initial_pos(axis, 3);
        });
    admittance_controller_->setPositionUpdate(
        [robot, initial_pos, axis](double pos) -> int {
          auto new_pose = initial_pos;
          new_pose(axis, 3) = pos + initial_pos(axis, 3);
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
  int axis_; //轴
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
    // force_sensor->bindPoseDetector([robot]() { return robot->currentPose();
    // });
    auto I3 = Eigen::Matrix3d::Identity();
    // force sensor filter

    // config controller
    // m,kv,K
    admittance_controller_ =
        std::make_shared<ControllerType>(1000 * I3, 2000 * I3, 1000 * I3);
    admittance_controller_->setDesiredPos(Eigen::Vector3d::Zero());
    admittance_controller_->setDesiredForce(Eigen::Vector3d::Zero());
    admittance_controller_->setForceSensor([force_sensor,
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
      std::cout << "t : " << t << " force : " << force.transpose() << std::endl;
      return force;
    });
    // 初始化虚拟传感器
    admittance_controller_->setVirtualPositionSensor(Eigen::Vector3d::Zero());
    auto initial_pos = robot_->currentPose();
    // admittance_controller_->setPositionSensor(
    //     [robot, initial_pos]() -> Eigen::Vector3d {
    //       // return robot->currentPose().block<3, 1>(0, 3) -
    //       //        initial_pos.block<3, 1>(0, 3);
    //     });
    admittance_controller_->setPositionUpdate([robot, initial_pos](
                                                  Eigen::Vector3d pos) -> int {
      auto new_pose = initial_pos;
      static auto start = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      double t =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
              .count() /
          1000.0;
      new_pose.block<3, 1>(0, 3) = pos + initial_pos.block<3, 1>(0, 3);
      std::cout << "t : " << t << " xyz : " << pos.transpose() << std::endl;
      Eigen::VectorXd joint =
          robot->topology()->trans_inv(new_pose, robot->currentJointState());
      std::cout << "t : " << t << " joint : " << joint.transpose() << std::endl;
      std::cout
          << "t : " << t << " actual_xyz : "
          << (robot->currentPose() - initial_pos).block<3, 1>(0, 3).transpose()
          << std::endl;
      std::cout << "t : " << t
                << " actual_joint : " << robot->currentJointState().transpose()
                << std::endl;
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
  std::shared_ptr<DownSampleFilter> downsample_filter_;
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