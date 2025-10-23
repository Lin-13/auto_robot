#include <aubo/aubo_robot.h>
#include <filesystem>
#include <fmt/format.h>
#include <iostream>
#include <optitrack/optitrack.h>
#include <utils/matrix_utils.h>
// adjust 函数需要每次标定时根据实际情况进行调整
auto T_bc_adjust = [](Eigen::Matrix3d &R) {
  // adjustRotateInplace(R, 1, 0, 0);
  // optitrack X轴与机器人X轴一致，Y轴向上
  if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitX()) < 0) {
    R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0);
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
  }
  if (R.block<3, 1>(0, 1).dot(Eigen::Vector3d::UnitZ()) < 0) {
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1);
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
  }
};
auto T_et_adjust = [](Eigen::Matrix3d &R) {
  // adjustRotateInplace(R, -1, 0, -1);
  // 调整target z轴，使其在end x轴正方向 ->
  // 机器人在末端沿X+运动时，target沿Z+运动
  if (R.block<3, 1>(0, 2).dot(Eigen::Vector3d::UnitX()) < 0) {
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1);
  }
  // 调整target x轴，使其在end z轴正方向 ->
  // 机器人在末端沿Z+运动时，target沿X+运动
  if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitZ()) < 0) {
    R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0);
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1);
  }
};
void calib_handeye_optitrack(const std::string &robot_name,
                             std::string target_name,
                             std::string data_folder = "", int num_poses = 10) {
  // 初始化 OptiTrack
  std::vector<std::string> rigid_body_names = {target_name};
  // std::string motive_ip = "192.168.100.103";
  const std::string motive_ip = "192.168.1.172";
  OptiTrackRigidBodyCap optitrack(rigid_body_names, motive_ip);
  Eigen::Matrix4d T_cam2target = optitrack.GetTransformcam2target(target_name);
  // 初始化机器人
  std::shared_ptr<Robot> robot;
  // std::string data_folder;
  if (robot_name == "left") {
    robot = auboRobotLeft();
    if (data_folder.empty()) {
      data_folder = "optitrack_handeye_left";
    }
  } else if (robot_name == "right") {
    robot = auboRobotRight();
    if (data_folder.empty()) {
      data_folder = "optitrack_handeye_right";
    }
  } else {
    fmt::print("Robot type {} not supported.\n", robot_name);
    return;
  }
  // std::unique_ptr<Robot> robot_right = auboRobotRight();
  // 尝试启动机器人
  try {
    if (robot->start(30ms) != 0) {
      fmt::print("Aubo robot start failed.\n");
    }
  } catch (const std::exception &e) {
    fmt::print("Aubo robot Error: {}\n", e.what());
  }
  // int num_poses = 10;
  std::vector<Eigen::Matrix4d> T_cam2target_list, T_base2gripper_list;
  T_cam2target_list.reserve(num_poses);
  T_base2gripper_list.reserve(num_poses);

  if (!std::filesystem::exists(data_folder)) {
    std::filesystem::create_directories(data_folder);
  }
  for (int i = 0; i < num_poses; ++i) {
    fmt::print("====================================\n");
    fmt::print("采集第 {} 个位姿,按下回车键进行采集", i + 1);
    std::cin.get();
    Eigen::MatrixXd T_cam2target =
        optitrack.GetTransformcam2target(target_name);
    T_cam2target_list.push_back(T_cam2target);
    std::cout << "采集到的位姿: \n" << T_cam2target << std::endl;
    std::cout << " RPY: "
              << RotToRPY(T_cam2target.block<3, 3>(0, 0)).transpose() * 180 /
                     M_PI
              << std::endl;
    Eigen::Matrix4d T_base2gripper = robot->currentPose();
    T_base2gripper_list.push_back(T_base2gripper);
    std::cout << "基座到末端的位姿: \n" << T_base2gripper << std::endl;
    std::cout << " RPY: "
              << RotToRPY(T_base2gripper.block<3, 3>(0, 0)).transpose() * 180 /
                     M_PI
              << std::endl;
    // 保存到文件
    std::cout << "Accept this pose? (y/n): ";
    if (std::cin.get() != 'y') {
      fmt::print("\t取消\n");
      T_cam2target_list.pop_back();
      T_base2gripper_list.pop_back();
      --i;
      while (std::cin.get() != '\n') {
        continue;
      }
      continue;
    }
    while (std::cin.get() != '\n') {
      continue;
    }
    writeEigenXdToFile(fmt::format("{}/cam2target_{}.txt", data_folder, i),
                       T_cam2target);
    writeEigenXdToFile(fmt::format("{}/base2gripper_{}.txt", data_folder, i),
                       T_base2gripper);
  }
  double res = 0;
  std::vector<Eigen::Matrix4d> T = calibrationHandtoEye(
      T_cam2target_list, T_base2gripper_list, T_bc_adjust, T_et_adjust, &res);
  fmt::print("Calibration residual - se3距离度量(平方范数): {}\n",
             sqrt(res / 2));
  std::cout << "T_base2camera result: \n" << T[0] << std::endl;
  std::cout << "T_end2target result: \n" << T[1] << std::endl;
  writeEigenXdToFile(fmt::format("{}/T_bc.txt", data_folder), T[0]);
  writeEigenXdToFile(fmt::format("{}/T_et.txt", data_folder), T[1]);
  return;
}
void calib_replay(std::string data_folder) {
  if (!std::filesystem::exists(data_folder)) {
    std::cout << "数据文件夹不存在: " << data_folder << std::endl;
    return;
  }
  int num_poses = 10;
  std::vector<Eigen::Matrix4d> T_cam2target_list, T_base2gripper_list;
  T_cam2target_list.reserve(num_poses);
  T_base2gripper_list.reserve(num_poses);
  for (int i = 0; i < num_poses; ++i) {
    // 从文件读取位姿
    Eigen::MatrixXd T_cam2target = readEigenXdFromFile(
        fmt::format("{}/cam2target_{}.txt", data_folder, i));
    T_cam2target_list.push_back(T_cam2target);
    std::cout << "Optitrack采集到的位姿: \n" << T_cam2target << std::endl;
    std::cout << " RPY: "
              << RotToRPY(T_cam2target.block<3, 3>(0, 0)).transpose() * 180 /
                     M_PI
              << std::endl;
    Eigen::Matrix4d T_base2gripper = readEigenXdFromFile(
        fmt::format("{}/base2gripper_{}.txt", data_folder, i));
    T_base2gripper_list.push_back(T_base2gripper);
    std::cout << "机器人基座到末端的位姿: \n" << T_base2gripper << std::endl;
    std::cout << " RPY: "
              << RotToRPY(T_base2gripper.block<3, 3>(0, 0)).transpose() * 180 /
                     M_PI
              << std::endl;
  }
  double res = 0;
  std::vector<Eigen::Matrix4d> T = calibrationHandtoEye(
      T_cam2target_list, T_base2gripper_list, T_bc_adjust, T_et_adjust, &res);
  fmt::print("Calibration residual - 距离度量(平方范数): {}\n", sqrt(res / 2));
  std::cout << "T_base2camera result: \n" << T[0] << std::endl;
  std::cout << "T_end2target result: \n" << T[1] << std::endl;
  return;
}
int main(int argc, char **argv) {
  // calib_handeye_optitrack("left", "target_left", "optitrack_handeye_left");
  calib_handeye_optitrack("right", "target_right", "optitrack_handeye_right");
  // calib_replay("optitrack_handeye_left");
  // calib_replay("optitrack_handeye_right");
  return 0;
}