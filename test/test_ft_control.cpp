#include "aubo/aubo_robot.h"
#include "hybrid_control/hybrid_control.h"
#include <context_monitor/monitor_server.h>
#include <fmt/format.h>
#include <iostream>
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
  ft_sensor->setBias();
  // ft_sensor->setRDTOutputRate(500);
  // testFTSensor(ft_sensor);
  auto tree = std::make_shared<TransformTree>();
  auto robot = auboRobotRight();
  robot->start(30ms); // TODO : 设置的时间与实际执行的时间不同
  auto ft_gravity_compensation = std::make_shared<FTSensorGravityCompensation>(
      ft_sensor, "gravity_compensation/right.txt");
  // auto hybrid_control =
  //     std::make_shared<BaseController>(robot, ft_gravity_compensation, tree);
  auto hybrid_control =
      std::make_shared<BaseController3d>(robot, ft_gravity_compensation, tree);
  auto start_time = std::chrono::steady_clock::now();
  int controller_timer_count = robot->controller()->timer_count;
  for (int i = 0;; i++) {
    auto start = std::chrono::steady_clock::now();
    hybrid_control->updateOnce();
    std::this_thread::sleep_until(start + 30ms);

    // monitor
    if (i % 100 == 0) {
      std::cout << "===========================================" << std::endl;
      auto end_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end_time - start_time)
                          .count();
      double freq = static_cast<double>(i) / duration * 1e3;
      int now_controller_timer_count = robot->controller()->timer_count;
      double controller_freq = static_cast<double>(now_controller_timer_count -
                                                   controller_timer_count) /
                               duration * 1e3;
      // std::cout << "Robot Pose:" << robot->currentPose() << std::endl;
      fmt::print("t: {:.3f}, preq:{:.3f} Hz, avg time : {:.3f} ms\n",
                 static_cast<double>(duration) / 1e3, freq, 1.0 / freq);
      std::cout << "F: "
                << ft_gravity_compensation->getCompensatedWrench().head<3>()
                << std::endl;
      fmt::print("controller freq: {:.3f} Hz,count:{}\n", controller_freq,
                 now_controller_timer_count - controller_timer_count);
      i = 0;
      start_time = std::chrono::steady_clock::now();
      controller_timer_count = robot->controller()->timer_count;
    }
  }
  return 0;
}