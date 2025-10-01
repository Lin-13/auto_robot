#include <aubo/aubo_robot.h>
#include <filesystem>
#include <fmt/format.h>
#include <iostream>
#include <optitrack/optitrack.h>
#include <utils/matrix_utils.h>
int main(int argc, char **argv) {
  // 初始化 OptiTrack
  std::vector<std::string> rigid_body_names = {"target"};
  std::string motive_ip = "192.168.100.103";
  OptiTrackRigidBodyCap optitrack(rigid_body_names, motive_ip);
  Eigen::Matrix4d T_cam2target = optitrack.GetTransformcam2target("target");
  // 初始化机器人
  std::unique_ptr<Robot> robot = auboRobotLeft();
  // std::unique_ptr<Robot> robot_right = auboRobotRight();
  // 尝试启动机器人
  try {
    if (robot->start(30ms) != 0) {
      fmt::print("Aubo robot start failed.\n");
    }
  } catch (const std::exception &e) {
    fmt::print("Aubo robot Error: {}\n", e.what());
  }
  int num_poses = 10;
  std::vector<Eigen::Matrix4d> T_cam2target_list, T_base2gripper_list;
  T_cam2target_list.reserve(num_poses);
  T_base2gripper_list.reserve(num_poses);
  std::string data_folder = "optitrack_handeye_left";
  if (!std::filesystem::exists(data_folder)) {
    std::filesystem::create_directories(data_folder);
  }
  for (int i = 0; i < num_poses; ++i) {
    fmt::print("采集第 {} 个位姿,按下回车键进行采集\n", i + 1);
    std::cin.get();
    Eigen::MatrixXd T_cam2target = optitrack.GetTransformcam2target("target");
    T_cam2target_list.push_back(T_cam2target);
    std::cout << "采集到的位姿: " << T_cam2target << std::endl;
    Eigen::Matrix4d T_base2gripper = robot->currentPose();
    T_base2gripper_list.push_back(T_base2gripper);
    std::cout << "基座到末端的位姿: " << T_base2gripper << std::endl;
    // 保存到文件
    writeEigenXdToFile(fmt::format("{}/cam2target_{}.txt", data_folder, i),
                       T_cam2target);
    writeEigenXdToFile(fmt::format("{}/base2gripper_{}.txt", data_folder, i),
                       T_base2gripper);
  }
  auto T_bc_adjust = [](Eigen::Matrix3d &R) {
    adjustRotateInplace(R, 1, 0, 1);
  };
  auto T_et_adjust = [](Eigen::Matrix3d &R) {
    adjustRotateInplace(R, -1, 0, -1);
  };
  double res = 0;
  std::vector<Eigen::Matrix4d> T = calibrationHandtoEye(
      T_cam2target_list, T_base2gripper_list, T_bc_adjust, T_et_adjust, &res);
  fmt::print("Calibration result: {}\n", res);
  std::cout << "T_base2camera result: \n" << T[0] << std::endl;
  std::cout << "T_end2target result: \n" << T[1] << std::endl;
  writeEigenXdToFile(fmt::format("{}/T_bc.txt", data_folder), T[0]);
  writeEigenXdToFile(fmt::format("{}/T_et.txt", data_folder), T[1]);
  return 0;
}