/**
 * @file calib_handeye_optitrack.cpp
 * @author your name (you@domain.com)
 * @brief T_bc : base 到 camera 的刚体变换,base视角下的camera位姿
 *        p_b = T_bc * p_c
 *        T_et : end 到 target 的刚体变换,end视角下的target位姿
 *        p_e = T_et * p_t
 *        使用OptiTrack进行手眼标定
 *
 * @copyright Copyright (c) 2025
 *
 */
// ! 标定结果 ! 2025/10/30
// * left : T_bc,T_et
// -0.0383915 -0.00421437 0.999254 -0.633543
// 0.998945 0.0250622 0.0384853 -0.995683
// -0.0252057 0.999677 0.00324775 0.203594
// 0 0 0 1
// RPY: 89.8139 1.44433 92.2009
// 0.561549 0.826826 -0.0319707 -0.0179136
// 0.823926 -0.562303 -0.0704434 -0.0664494
// -0.0762217 0.0132159 -0.997003 0.145208
// 0 0 0 1
// RPY: 179.241  4.3714 55.7235
// * right : T_bc,T_et
// -0.12685 -0.00521243 0.991908 0.243538
// 0.991296 0.0348556 0.126955 -1.00443
// -0.0352353 0.999379 0.00074562 0.207359
// 0 0 0 1
// RPY: 89.9573 2.01925 97.2922
// 0.754443 -0.651168 0.0824413 -0.0459264
// 0.651892 0.758006 0.0215253 -0.0146394
// -0.0765076 0.0375032 0.996363 0.187496
// 0 0 0 1
// RPY:  2.1556 4.38785 40.8293
// ! 标定结果 2025/10/31 15:05
// ? left
// Calibration residual - se3距离度量(平方范数): 0.012740872273113277
// * T_base2camera result:
// -0.00111437 -0.00740136    0.999972   -0.808279
//    0.999955 -0.00942824  0.00104457   -0.991214
//  0.00942025    0.999928  0.00741153    0.195133
//           0           0           0           1
//  RPY:   89.5753 -0.539748   90.0639
// * T_end2target result:
//    0.748199    0.660653  -0.0611139  -0.0195971
//    0.658647   -0.750689  -0.0514807  -0.0670499
//  -0.0798884 -0.00173463   -0.996802    0.146602
//           0           0           0           1
//  RPY:  -179.9 4.58215 41.3578
// ? right
// Calibration residual - se3距离度量(平方范数): 0.00931482015346127
// *T_base2camera result:
//   -0.065261 -0.00627182    0.997849     0.06846
//    0.997854 -0.00569689   0.0652255   -0.963386
//  0.00527555    0.999964  0.00663014    0.206048
//           0           0           0           1
//  RPY:   89.6201 -0.302268   93.7419
// *T_end2target result:
//   0.653653  -0.754371  0.0605111  -0.046036
//   0.718262   0.643575   0.264407 -0.0145502
//  -0.238405  -0.129368   0.962511   0.188414
//          0          0          0          1
//  RPY: -7.65506  13.7924  47.6963
#include <aubo/aubo_robot.h>
#include <filesystem>
#include <fmt/format.h>
#include <iostream>
#include <optitrack/optitrack.h>
#include <utils/matrix_utils.h>
// adjust 函数需要每次标定时根据实际情况进行调整
// auto T_bc_adjust = [](Eigen::Matrix3d &R) {
//   // adjustRotateInplace(R, 1, 0, 0);
//   // optitrack X轴与机器人X轴一致，Y轴向上
//   if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitX()) < 0) {
//     R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0);
//     R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
//   }
//   if (R.block<3, 1>(0, 1).dot(Eigen::Vector3d::UnitZ()) < 0) {
//     R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1);
//     R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
//   }
// };
// auto T_et_adjust = [](Eigen::Matrix3d &R) {
//   // adjustRotateInplace(R, -1, 0, -1);
//   // 调整target z轴，使其在end x轴正方向 ->
//   // 机器人在末端沿X+运动时，target沿Z+运动
//   if (R.block<3, 1>(0, 2).dot(Eigen::Vector3d::UnitX()) < 0) {
//     R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
//     R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1);
//   }
//   // 调整target x轴，使其在end z轴正方向 ->
//   // 机器人在末端沿Z+运动时，target沿X+运动
//   if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitZ()) < 0) {
//     R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0);
//     R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1);
//   }
// };
// * 双臂夹取时的位姿为准进行标定
// * 先将机器人设置为准备夹取的姿态,然后再optitrack中固定刚体坐标系的姿态
// 以准备夹取姿态在Motive中设置target_left刚体后的调整函数
auto T_bc_adjust = [](Eigen::Matrix3d &R) {
  // adjustRotateInplace(R, 1, 0, 0);
  // optitrack X轴与机器人base的Y轴对齐
  if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitY()) < 0) {
    R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0);
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2);
  }
  // optitrack Y轴与机器人Z轴对齐 (向上)
  if (R.block<3, 1>(0, 1).dot(Eigen::Vector3d::UnitZ()) < 0) {
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1); // Y
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2); // Z
  }
};
auto T_et_adjust_left = [](Eigen::Matrix3d &R) {
  // 末端坐标系X轴与刚体坐标系同向
  if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitX()) < 0) {
    R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0); // X
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1); // Y
  }
  // 末端坐标系Z轴沿刚体坐标系Z轴反方向
  if (R.block<3, 1>(0, 2).dot(Eigen::Vector3d::UnitZ()) > 0) {
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2); // Z
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1); // Y
  }
};
auto T_et_adjust_right = [](Eigen::Matrix3d &R) {
  // 末端坐标系X轴与刚体坐标系同向
  if (R.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitX()) < 0) {
    R.block<3, 1>(0, 0) = -R.block<3, 1>(0, 0); // X
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1); // Y
  }
  // 末端坐标系Z轴沿刚体坐标系Z轴正方向
  if (R.block<3, 1>(0, 2).dot(Eigen::Vector3d::UnitZ()) < 0) {
    R.block<3, 1>(0, 2) = -R.block<3, 1>(0, 2); // Z
    R.block<3, 1>(0, 1) = -R.block<3, 1>(0, 1); // Y
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
  std::function<void(Eigen::Matrix3d & R)> T_et_adjust;
  if (robot_name == "left") {
    robot = auboRobotLeft();
    T_et_adjust = T_et_adjust_left;
    if (data_folder.empty()) {
      data_folder = "optitrack_handeye_left";
    }
  } else if (robot_name == "right") {
    robot = auboRobotRight();
    T_et_adjust = T_et_adjust_right;
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
  std::cout << " RPY: "
            << RotToRPY(T[0].block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  std::cout << "T_end2target result: \n" << T[1] << std::endl;
  std::cout << " RPY: "
            << RotToRPY(T[1].block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  writeEigenXdToFile(fmt::format("{}/T_bc.txt", data_folder), T[0]);
  writeEigenXdToFile(fmt::format("{}/T_et.txt", data_folder), T[1]);
  return;
}
void calib_replay(std::string robot_name, std::string data_folder) {
  std::function<void(Eigen::Matrix3d & R)> T_et_adjust;
  if (robot_name == "left") {
    T_et_adjust = T_et_adjust_left;
  } else if (robot_name == "right") {
    T_et_adjust = T_et_adjust_right;
  } else {
    fmt::print("Robot type {} not supported.\n", robot_name);
    return;
  }
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
  std::cout << " RPY: "
            << RotToRPY(T[0].block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  std::cout << "T_end2target result: \n" << T[1] << std::endl;
  std::cout << " RPY: "
            << RotToRPY(T[1].block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  writeEigenXdToFile(fmt::format("{}/T_bc.txt", data_folder), T[0]);
  writeEigenXdToFile(fmt::format("{}/T_et.txt", data_folder), T[1]);
  return;
}
int main(int argc, char **argv) {
  // calib_handeye_optitrack("left", "target_left", "optitrack_handeye_left",
  // 5);
  calib_handeye_optitrack("right", "target_right", "optitrack_handeye_right",
                          5);
  // * replay
  // calib_replay("left", "optitrack_handeye_left");
  // calib_replay("right", "optitrack_handeye_right");
  return 0;
}