#include "aubo/aubo_controller.h"
#include "aubo/aubo_robot.h"
#include "utils/matrix_utils.h"
// #define SERVER_HOST_left "192.168.1.131"
// #define SERVER_PORT_left 8899

// #define SERVER_HOST_right "192.168.1.101"
// #define SERVER_PORT_right 8899
// void testAubo() {
//   AuboController aubo_controller(SERVER_HOST_left, SERVER_PORT_left, "aubo",
//                                  "1");
//   aubo_controller.Initialize();
//   // RobotController::RobotJointState joint_state;
//   // joint_state.joints = 6;
//   // joint_state.joint_state = Eigen::VectorXd::Ones(6);
//   // joint_state.timestamp = std::chrono::steady_clock::now();
//   // aubo_controller.setJointState(joint_state);
//   auto joint_state = aubo_controller.getJointState();
//   std::cout << "joint state: " << joint_state.joint_state.transpose()
//             << std::endl;
//   return;
// }
// 测试时请确保与机器人的连接正常
void testAuboRobot() {
  // auto aubo_robot = auboRobotLeft();
  auto aubo_robot = auboRobotRight(1s, ControllerConfig({k_p : 1, k_v : 0.1}));
  // aubo_robot.start(1s);
  auto start = std::chrono::steady_clock::now();
  auto pose = aubo_robot.currentPose();
  auto end = std::chrono::steady_clock::now();
  std::cout << "connect delay: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     start)
                   .count()
            << " us" << std::endl; // 347us

  std::cout << "joint: "
            << aubo_robot.currentJointState().transpose() * 180 / M_PI
            << std::endl;
  std::cout << "pose: \n" << pose << std::endl;
  // std::cout << "rpy pose: \n"
  //           << pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0).transpose() * 180
  //           /
  //                  M_PI
  //           << " " << pose.block<3, 1>(0, 3).transpose() << std::endl;
  Robot::Trajectory pose_trajectory;
  auto T1 =
      HomoMatrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0, -0.05));
  auto T2 =
      HomoMatrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.0, 0, 0.05));
  pose_trajectory.emplace_back(0, Eigen::Matrix4d::Identity());
  pose_trajectory.emplace_back(2, T1);
  pose_trajectory.emplace_back(4, T2);
  pose_trajectory.emplace_back(6, Eigen::Matrix4d::Identity());
  int ret = aubo_robot.start(20ms);
  ret = aubo_robot.MovePoseRelative(pose_trajectory, 20ms, 0);
  if (ret != 0) {
    std::cout << "Trajectory Move failed" << std::endl;
    return;
  }
  std::this_thread::sleep_for(8s);
  aubo_robot.stop();
  // auto result = aubo_robot.controller()->getJointState();
  std::cout << "Last JointState (deg): "
            << aubo_robot.currentJointState().transpose() * 180 / M_PI
            << std::endl;
  std::cout << "Last Pose: \n" << aubo_robot.currentPose() << std::endl;
  // 有概率报错
  // *** buffer overflow detected ***: terminated
  return;
}
int main() {
  testAuboRobot();
  return 0;
}