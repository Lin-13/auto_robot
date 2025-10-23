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
    // force_sensor->getSensor()->setBias();
    admittance_controller_ = std::make_shared<ControllerType>(10, 100, 1000);
    admittance_controller_->setDesiredPos(0);
    admittance_controller_->setDesiredForce(0);
    admittance_controller_->setForceSensor([force_sensor]() -> double {
      return force_sensor->getCompensatedWrench().z();
    });
    auto initial_pos = robot_->currentPose();
    admittance_controller_->setPositionSensor([robot, initial_pos]() -> double {
      return robot->currentPose()(2, 3) - initial_pos(2, 3); // z
    });
    admittance_controller_->setPositionUpdate([robot,
                                               initial_pos](double pos) -> int {
      auto new_pose = initial_pos;
      new_pose(2, 3) = pos + initial_pos(2, 3);
      std::cout << "current pos:" << robot->currentPose()(2, 3) << std::endl;
      std::cout << "init pos:" << initial_pos(2, 3) << std::endl;
      std::cout << "set pose to:" << pos << "\n" << new_pose << std::endl;
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