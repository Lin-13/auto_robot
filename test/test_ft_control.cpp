/**
 * @file test_ft_control.cpp
 * @author your name (you@domain.com)
 * @brief 机器人导纳控制测试
 * @version 0.1
 * @date 2025-10-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "aubo/aubo_controller.h"
#include "aubo/aubo_robot.h"
#include "hybrid_control/hybrid_control.h"
#include <context_monitor/monitor_server.h>
#include <fmt/format.h>
#include <iostream>
#include <utils/debug_utils.h>
void testFTSensor(std::shared_ptr<ati::FTSensor> ft_sensor) {
  std::vector<double> force(6, 0);
  auto test_start = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000; i++) {
    ft_sensor->getMeasurements(force.data());
  }
  auto test_end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                      test_end - test_start)
                      .count();
  fmt::print("测试耗时: {:.3f} ms\n", static_cast<double>(duration));
  std::cout << "传感器实际频率：" << ft_sensor->getRDTRate() << "Hz"
            << std::endl;
}
// TODO : 机器人在导纳控制下会出现运动摇摆，难以通过调节导纳参数解决
int main() {
  auto ft_sensor = std::make_shared<ati::FTSensor>();
  ft_sensor->init(RIGHT_ATI_IP);
  // ft_sensor->setBias();
  // ft_sensor->setRDTOutputRate(500);
  auto tree = std::make_shared<TransformTree>();
  auto robot = auboRobotRight(30ms);
  std::shared_ptr<AuboController> controller =
      std::dynamic_pointer_cast<AuboController>(robot->controller());
  // controller->enable_log_ = 1;
  robot->start(30ms); // TODO : 设置的时间与实际执行的时间不同
  auto ft_gravity_compensation = std::make_shared<FTSensorGravityCompensation>(
      ft_sensor, "gravity_compensation/right.txt");
  // right R_sensor :
  Eigen::Matrix3d R_sensor =
      Eigen::AngleAxisd(15.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  // left R_sensor :
  // Eigen::Matrix3d R_sensor =
  //     Eigen::AngleAxisd(-165.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
  //         .toRotationMatrix();
  // 重力补偿的位姿检测函数
  ft_gravity_compensation->bindPoseDetector([robot, R_sensor]() {
    Eigen::Matrix3d R = robot->currentPose().block<3, 3>(0, 0);
    R = R * R_sensor;
    return R;
  });
  // auto hybrid_control =
  //     std::make_shared<BaseController>(robot, ft_gravity_compensation, tree);
  auto hybrid_control =
      std::make_shared<BaseController3d>(robot, ft_gravity_compensation, tree);
  ft_gravity_compensation->setSoftBias();
  for (int i = 0;; i++) {
    PROFILE();
    auto start = std::chrono::steady_clock::now();
    hybrid_control->updateOnce(); // ****
    std::this_thread::sleep_until(start + 20ms);

    // monitor
    if (i % 100 == 0) {
      // std::cout << "Robot Pose:" << robot->currentPose() << std::endl;
      // PRINT_PROFILER();
      // std::cout << "F raw: " << ft_gravity_compensation->getWrench()
      //           << std::endl;
      // std::cout << "F: " << ft_gravity_compensation->getCompensatedWrench()
      //           << std::endl;
    }
  }
  return 0;
}