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
// ! 标定结果 2025/12/14 18:57
// ? left
// Calibration residual - se3距离度量(平方范数): 0.006920758214584341
// T_base2camera result:
//    0.057846  -0.0201965    0.998121    -1.02455
//    0.998326   0.0010928  -0.0578358   -0.597152
// 7.73301e-05    0.999795   0.0202259   -0.169806
//           0           0           0           1
//  RPY:     88.8411 -0.00443069     86.6838
// T_end2target result:
//   0.975496  -0.159286  -0.151774 -0.0268179
//  -0.156297  -0.987208  0.0314996 -0.0340043
//   -0.15485 -0.0070059  -0.987913    0.14375
//          0          0          0          1
//  RPY: -179.594  8.90808 -9.10274
// ? right
// Calibration residual - se3距离度量(平方范数): 0.004111004814341146
// T_base2camera result:
//   -0.103727 -0.00877426    0.994567    0.198589
//    0.994577  0.00664767    0.103786   -0.811461
// -0.00752221    0.999939  0.00803714   -0.152915
//           0           0           0           1
//  RPY:  89.5395 0.430995   95.954
// T_end2target result:
//   0.614777  -0.778497  -0.126456 -0.0757802
//   0.779289   0.624278  -0.054637 -0.0522052
//   0.121479 -0.0649565   0.990466   0.169822
//          0          0          0          1
// ! 标定结果 2025/12/15 12:01
// ? left
// Calibration residual - se3距离度量(平方范数): 0.0032637194097666954
// T_base2camera result:
//  0.0545734 -0.0212487   0.998284   -1.24862
//   0.998415  0.0149411 -0.0542625  -0.677085
// -0.0137625   0.999663  0.0220304    -0.1562
//          0          0          0          1
//  RPY:  88.7375 0.788556  86.8713
// T_end2target result:
//   0.974211  -0.161237  -0.157844 -0.0852167
//  -0.173864  -0.982302 -0.0696677 0.00227761
//  -0.143817  0.0953144  -0.985003   0.126744
//          0          0          0          1
//  RPY:  174.473  8.26879 -10.1188
// ? right
// Calibration residual - se3距离度量(平方范数): 0.0014908037337646228
// T_base2camera result:
//   -0.0938892    -0.011685     0.995514      0.13458
//     0.995582 -0.000181593    0.0938935    -0.959317
// -0.000916365     0.999932    0.0116504    -0.154987
//            0            0            0            1
//  RPY:   89.3325 0.0525038   95.3874
// T_end2target result:
//    0.75726  -0.652892  0.0170003 -0.0747535
//   0.651926   0.757196  0.0405762 -0.0549088
// -0.0393644 -0.0196438   0.999032   0.170732
//          0          0          0          1
//  RPY: -1.12645    2.256  40.7252
// ! 标定结果 2025/12/16 16:11
// ? left
// Calibration residual - se3距离度量(平方范数): 0.0033948326499661455
// T_base2camera result:
//    0.115276  -0.0182138    0.993167     -1.1464
//    0.993329  0.00518602     -0.1152   -0.612604
// -0.00305236    0.999821   0.0186901   0.0267181
//           0           0           0           1
//  RPY:  88.9291 0.174888  83.3804
// T_end2target result:
//   0.968399  -0.248487 -0.0213935 -0.0839098
//  -0.248795  -0.968466 -0.0131779 0.00376756
// -0.0174444  0.0180841  -0.999684   0.124882
//          0          0          0          1
//  RPY:  178.964 0.999538 -14.4085
// ? right
// Calibration residual - se3距离度量(平方范数): 0.006749183105734072
// T_base2camera result:
//  -0.0457999  -0.0137037    0.998857    0.242298
//     0.99895 -0.00152635   0.0457833   -0.902416
// 0.000897207    0.999905   0.0137592   0.0324013
//           0           0           0           1
//  RPY:    89.2116 -0.0514062    92.6251
// T_end2target result:
//    0.694464   -0.714767  -0.0826292  -0.0763711
//      0.7106    0.699344  -0.0772379  -0.0518208
//    0.112993 -0.00507739    0.993583    0.169536
//           0           0           0           1
// ! 标定结果 2026/1/5 13:09
// ? left
// Calibration residual - se3距离度量(平方范数): 0.007082939688700219
// T_base2camera result:
// -0.0103714  -0.023514    0.99967  -0.708714
//   0.999898 -0.0101059  0.0101361  -0.968161
// 0.00986422   0.999672  0.0236164  0.0410693
//          0          0          0          1
//  RPY:   88.6467 -0.565187   90.5943
// T_end2target result:
//   0.973396  -0.228275  0.0197789 -0.0819053
//  -0.228031  -0.973556 -0.0138377 0.00310702
//  0.0224147 0.00895931  -0.999709   0.126367
//          0          0          0          1
//  RPY:  179.487 -1.28437 -13.1845
// ? right
// Calibration residual - se3距离度量(平方范数): 0.004031565930251729
// T_base2camera result:
//   -0.118233  -0.0109776    0.992925    0.428906
//    0.992966  0.00499531    0.118293   -0.958891
// -0.00625853    0.999927   0.0103097   0.0588124
//           0           0           0           1
//  RPY: 89.4093 0.35859 96.7902
// T_end2target result:
//   0.682207  -0.702131   0.203977 -0.0759094
//   0.663624   0.711715    0.23036 -0.0515302
//  -0.306916  -0.021789   0.951487   0.170639
//          0          0          0          1
//  RPY: -1.31184  17.8735  44.2089
// ! 标定结果 2026/1/5 17:35
// ? left
// Calibration residual - se3距离度量(平方范数): 0.015621928829618997
// T_base2camera result:
//   0.143791 -0.0263586   0.989257  -0.750288
//   0.989375 -0.0178792  -0.144285  -0.914677
//  0.0214902   0.999493  0.0235076   0.272667
//          0          0          0          1
//  RPY:  88.6527 -1.23139  81.7308
// T_end2target result:
//   0.897939  -0.338726  -0.281016  -0.076097
//  -0.314253   -0.94047   0.129467 0.00592934
//  -0.308141  -0.027943   -0.95093   0.125948
//          0          0          0          1
//  RPY: -178.317  17.9473 -19.2886
// ? right
// Calibration residual - se3距离度量(平方范数): 0.012841900949888926
// T_base2camera result:
//  -0.0535638 -0.00588172    0.998547    0.524192
//    0.998544 -0.00678363   0.0535237   -0.951391
//  0.00645896     0.99996  0.00623651    0.302121
//           0           0           0           1
//  RPY:   89.6427 -0.370074   93.0705
// T_end2target result:
//   0.668238  -0.714394   0.207604 -0.0733387
//    0.67108   0.699288   0.246269 -0.0508686
//  -0.321108 -0.0252477   0.946706   0.172881
//          0          0          0          1
//  RPY: -1.52766    18.73  45.1216
// ! 标定结果 2026/1/5 20:52
// ? left
// Calibration residual - se3距离度量(平方范数): 0.006269950656057712
// T_base2camera result:
//   0.0149317  -0.0333887    0.999331    -0.71045
//    0.999887  0.00247015  -0.0148575   -0.944962
// -0.00197243    0.999439   0.0334218    0.287248
//           0           0           0           1
//  RPY:  88.0847 0.113012  89.1444
// T_end2target result:
//    0.939711   -0.284467   -0.189795  -0.0791341
//   -0.275245   -0.958532   0.0738697 -0.00189191
//   -0.202938  -0.0171761   -0.979041    0.119796
//           0           0           0           1
//  RPY: -178.995  11.7088 -16.3255
// ? right
// Calibration residual - se3距离度量(平方范数): 0.010210397565805595
// T_base2camera result:
//  -0.0641551 -0.00313021    0.997935    0.379287
//    0.997938  0.00152863   0.0641601    -1.02312
//  -0.0017263    0.999994  0.00302569    0.302833
//           0           0           0           1
//  RPY: 89.8266 0.09891 93.6783
// T_end2target result:
//   0.709176  -0.702761  0.0565375 -0.0738531
//   0.701832   0.711315  0.0382517 -0.0505037
// -0.0670977  0.0125526   0.997667   0.174601
//          0          0          0          1
//  RPY: 0.720856  3.84731  44.701
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
  // left handeye calibration
  // calib_handeye_optitrack("left", "target_left", "optitrack_handeye_left",
  // 5);

  // right handeye calibration
  calib_handeye_optitrack("right", "target_right", "optitrack_handeye_right",
                          5);

  // * replay
  // calib_replay("left", "optitrack_handeye_left");
  // calib_replay("right", "optitrack_handeye_right");
  return 0;
}