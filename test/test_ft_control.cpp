#include "aubo/aubo_robot.h"
#include "hybrid_control/hybrid_control.h"
#include <context_monitor/monitor_server.h>
#include <iostream>
int main() {
  auto robot = auboRobotRight();
  robot->start(10ms);
  auto ft_sensor = std::make_shared<ati::FTSensor>();
  auto tree = std::make_shared<TransformTree>();
  ft_sensor->init(RIGHT_ATI_IP);
  auto ft_gravity_compensation = std::make_shared<FTSensorGravityCompensation>(
      ft_sensor, "gravity_compensation/right.txt");
  auto hybrid_control =
      std::make_shared<BaseController>(robot, ft_gravity_compensation, tree);
  while (1) {
    hybrid_control->updateOnce();
    std::cout << "F: "
              << ft_gravity_compensation->getCompensatedWrench().head<3>()
              << std::endl;
    // std::cout << "Robot Pose:" << robot->currentPose()(2, 3) << std::endl;
    hybrid_control->getAdmittanceController();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}