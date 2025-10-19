#include "aubo/aubo_robot.h"
#include "optitrack/optitrack.h"
#include "robot_interface/robot.h"
#include "utils/matrix_utils.h"
#include "utils/utils.h"
#include <fmt/format.h>
// #define CONTEXT_MONITOR
#ifdef CONTEXT_MONITOR
#include "context_monitor/monitor_server.h"
#endif
/**
 * @brief 计算从T_ct_start到T_ct_end的移动矩阵
 *        原理：`T_bc * T_ct = T_be * T_et`
 * @param T_bc 机器人基到相机的变换矩阵 (机器人坐标系中的相机位姿)
 * @param T_et 末端到刚体目标的变换矩阵
 * @param T_ct_start 相机到目标的初始变换矩阵 (相机坐标系中的初始位姿)
 * @param T_ct_end 相机到目标的目标变换矩阵 (相机坐标系中的目标位姿)
 * @return 机器人基坐标系下的相对变换
 *          `T_be_end = T_ret * T_be_start`
 */
Eigen::MatrixXd GlobalMoveToEndMove(Eigen::MatrixXd T_bc, Eigen::MatrixXd T_et,
                                    Eigen::MatrixXd T_ct_start,
                                    Eigen::MatrixXd T_ct_end) {
  // 计算T_be_start
  Eigen::MatrixXd T_be_start =
      T_et.transpose()
          .jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
          .solve(T_ct_start.transpose() * T_bc.transpose())
          .transpose();
  Eigen::MatrixXd T_be_end =
      T_et.transpose()
          .jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
          .solve(T_ct_end.transpose() * T_bc.transpose())
          .transpose();
  Eigen::MatrixXd T_move_rel = T_be_start.inverse() * T_be_end;
  Eigen::MatrixXd T_move = T_be_start * T_move_rel * T_be_start.inverse();
  // Eigen::MatrixXd T_move = T_be_end * T_be_start.inverse();
  // debug
  // fmt::print("\e[1;33mGlobalMoveToEndMove\e[0m::T_be_start: \n{}\n",
  //            T_be_start);
  // fmt::print("RPY:{}\n",
  //            RotToRPY(T_be_start.block<3, 3>(0, 0)).transpose() * 180 /
  //            M_PI);
  // fmt::print("\e[1;33mGlobalMoveToEndMove\e[0m::T_be_end: \n{}\n", T_be_end);
  // fmt::print("RPY:{}\n",
  //            RotToRPY(T_be_end.block<3, 3>(0, 0)).transpose() * 180 / M_PI);
  // fmt::print("\e[1;33mGlobalMoveToEndMove\e[0m::T_move: \n{}\n", T_move);
  // fmt::print("RPY:{}\n",
  //            RotToRPY(T_move.block<3, 3>(0, 0)).transpose() * 180 / M_PI);
  // fmt::print("Check\n{}", (T_move * T_be_start - T_be_end).norm());
  return T_move;
}
int main(int argc, char *argv[]) {
  fmt::print("\e[1;35m========Optitrack & Robot Initialize========\e[0m\n");
  std::vector<std::string> rigid_body_names = {"target_left", "target_right",
                                               "object"};
  // std::string motive_ip = "192.168.100.103";
  const std::string motive_ip = "192.168.1.172";
  OptiTrackRigidBodyCap optitrack(rigid_body_names, motive_ip);
  Eigen::Matrix4d T_cam2target_left =
      optitrack.GetTransformcam2target("target_left");
  Eigen::Matrix4d T_cam2target_right =
      optitrack.GetTransformcam2target("target_right");
  Eigen::Matrix4d T_cam2object = optitrack.GetTransformcam2target("object");
  // 初始化机器人
  std::unique_ptr<Robot> robot_left = auboRobotLeft();
  std::unique_ptr<Robot> robot_right = auboRobotRight();
  try {
    if (robot_left->start(30ms) != 0) {
      fmt::print("Aubo robot left start failed.\n");
    }
    if (robot_right->start(30ms) != 0) {
      fmt::print("Aubo robot right start failed.\n");
    }
  } catch (const std::exception &e) {
    fmt::print("Aubo robot Error: {}\n", e.what());
  }
  // 读取标定矩阵
  fmt::print("\e[1;35m=========Hand Eye Calibration Data=========\e[0m\n");
  std::string left_calib_folder = "optitrack_handeye_left";
  auto T_left_bc = readEigenXdFromFile(left_calib_folder + "/T_bc.txt");
  auto T_left_et = readEigenXdFromFile(left_calib_folder + "/T_et.txt");
  if (T_left_bc.rows() != 4 || T_left_et.rows() != 4 || T_left_bc.cols() != 4 ||
      T_left_et.cols() != 4) {
    fmt::print("T_left_bc or T_left_et is not 4x4 matrix.\n");
    return -1;
  }
  fmt::print("T_left_bc : \n{}\n", T_left_bc);
  fmt::print("T_left_et : \n{}\n", T_left_et);
  std::string right_calib_folder = "optitrack_handeye_right";
  auto T_right_bc = readEigenXdFromFile(right_calib_folder + "/T_bc.txt");
  auto T_right_et = readEigenXdFromFile(right_calib_folder + "/T_et.txt");
  if (T_right_bc.rows() != 4 || T_right_et.rows() != 4 ||
      T_right_bc.cols() != 4 || T_right_et.cols() != 4) {
    fmt::print("T_right_bc or T_right_et is not 4x4 matrix.\n");
    return -1;
  }
  fmt::print("T_right_bc : \n{}\n", T_right_bc);
  fmt::print("T_right_et : \n{}\n", T_right_et);
  // 测试标定矩阵的MSE
  // 原理：// T_bc * T_ct = T_be * T_et
  int test_num = 2;
  double mse_left = 0;
  fmt::print("\e[1;35m========Robot Test==========\e[0m\n");
  fmt::print("Testing left robot ...\n");
  for (int i = 0; i < test_num; i++) {
    Eigen::MatrixXd T_left_be = robot_left->currentPose();
    Eigen::MatrixXd T_left_ct = optitrack.GetTransformcam2target("target_left");
    // 计算T_ct_est
    Eigen::MatrixXd T_left_ct_est =
        T_left_bc.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
            .solve(T_left_be * T_left_et);
    auto so3_error = SO3Toso3(T_left_ct.block<3, 3>(0, 0).transpose() *
                              T_left_ct_est.block<3, 3>(0, 0))
                         .squaredNorm();
    double t_error =
        (T_left_ct_est.block<3, 1>(0, 3) - T_left_ct.block<3, 1>(0, 3))
            .squaredNorm();
    double gamma = 1; // 权重参数
    mse_left += so3_error + gamma * t_error;
    fmt::print("T_left_ct: \n{}\nT_left_ct_est: \n{}\n", T_left_ct,
               T_left_ct_est);
    fmt::print("\033[1;33mso3_error: {:.6f} deg, t_error: {:.6f}mm\033[0m\n",
               std::sqrt(so3_error) * 180.0 / M_PI,
               std::sqrt(t_error) * 1000.0);
    std::this_thread::sleep_for(2s);
  }
  fmt::print("\033[1;33mMSE left: {:.6f}\033[0m\n",
             std::sqrt(mse_left / test_num));
  // 测试右机器人
  fmt::print("Testing right robot ...\n");
  double mse_right = 0;
  for (int i = 0; i < test_num; i++) {
    Eigen::MatrixXd T_right_be = robot_right->currentPose();
    Eigen::MatrixXd T_right_ct =
        optitrack.GetTransformcam2target("target_right");
    // 计算T_ct_est
    Eigen::MatrixXd T_right_ct_est =
        T_right_bc.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
            .solve(T_right_be * T_right_et);
    auto so3_error = SO3Toso3(T_right_ct.block<3, 3>(0, 0).transpose() *
                              T_right_ct_est.block<3, 3>(0, 0))
                         .squaredNorm();
    double t_error =
        (T_right_ct_est.block<3, 1>(0, 3) - T_right_ct.block<3, 1>(0, 3))
            .squaredNorm();
    double gamma = 1; // 权重参数
    mse_right += so3_error + gamma * t_error;
    fmt::print("T_right_ct: \n{}\nT_right_ct_est: \n{}\n", T_right_ct,
               T_right_ct_est);
    fmt::print("\033[1;33mso3_error: {:.6f} deg, t_error: {:.6f}mm\033[0m\n",
               std::sqrt(so3_error) * 180.0 / M_PI,
               std::sqrt(t_error) * 1000.0);
    std::this_thread::sleep_for(1s);
  }
  fmt::print("\033[1;33mMSE right: {:.6f}\033[0m\n",
             std::sqrt(mse_right / test_num));
  // ================================
  fmt::print("\e[1;35m================Control Start===============\e[0m\n");
  // 开始控制
  fmt::print(
      "Step 1 : Move both robot to desired position and align the object.\n");
  // 通过object生成T_ct_end
  // End到TCP坐标系的关系，
  // 当前由程序指定
  // TODO： 通过标定获得更加准确的TCP
  Eigen::MatrixXd T_end2tcp = Eigen::Matrix4d::Identity();
  T_end2tcp.block<3, 1>(0, 3) =
      Eigen::Vector3d(0, 0, 0.25); //通过测量得到大概关系
  auto T_tcp2end = T_end2tcp.inverse();

  // 状态容器初始化
  /********************************** */
  Eigen::MatrixXd T_object_current = optitrack.GetTransformcam2target("object");
  Eigen::MatrixXd T_left_ct_current =
      optitrack.GetTransformcam2target("target_left");
  Eigen::MatrixXd T_right_ct_current =
      optitrack.GetTransformcam2target("target_right");
  Eigen::MatrixXd T_left_be_current = robot_left->currentPose();
  Eigen::MatrixXd T_right_be_current = robot_right->currentPose();
  // 编程预设值 ，其和锥桶的直径等参数相关
  // TODO: 通过力信息建立连接，当前通过相对位置关系建立TCP
  // 需要注意optitrack坐标系+Y向上，ROBOT中+Z向上
  Eigen::Matrix4d T_left_tcp_rel = Eigen::Matrix4d::Identity();
  // T_left_tcp_rel.block<3, 3>(0, 0) = T_left_ct_current.block<3, 3>(0, 0);
  T_left_tcp_rel.block<3, 1>(0, 3) = Eigen::Vector3d(0.20, 0.25, 0);
  Eigen::Matrix4d T_right_tcp_rel = Eigen::Matrix4d::Identity();
  // T_right_tcp_rel.block<3, 3>(0, 0) = T_right_ct_current.block<3, 3>(0, 0);
  T_right_tcp_rel.block<3, 1>(0, 3) = Eigen::Vector3d(-0.20, 0.25, 0);
#if defined CONTEXT_MONITOR
  {
    fmt::print("Context monitor enabled.\n");
    // REGISTER_MONITOR_VARIABLE(T_object_current);
    // REGISTER_MONITOR_VARIABLE(T_left_ct_current);
    // REGISTER_MONITOR_VARIABLE(T_right_ct_current);
  }
  std::thread server_thread(RunMonitorServer, 50051);
#endif
  /********************************* */
  while (1) {
    Eigen::MatrixXd T_left_ct_des, T_right_ct_des;
    T_object_current = optitrack.GetTransformcam2target("object");
    T_object_current.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    // 物体姿态对齐到相机坐标系

    // T_tcp 相机坐标中的位姿
    // T_et * T_target_tcp = T_e_tcp
    // T_ct * T_target_tcp = T_tcp
    // => T_ct = T_tcp *  inv(T_et.inv * T_e_tcp) = T_tcp * T_e_tcp.inv * T_et
    Eigen::Matrix4d T_left_tcp = T_object_current * T_left_tcp_rel;
    Eigen::Matrix4d T_right_tcp = T_object_current * T_right_tcp_rel;
    T_left_ct_des = T_left_tcp * T_tcp2end * T_left_et;
    T_right_ct_des = T_right_tcp * T_tcp2end * T_right_et;
    // DEBUG ：强制des的方向与current相同，只移动位置
    T_left_ct_des.block<3, 3>(0, 0) = T_left_ct_current.block<3, 3>(0, 0);
    T_right_ct_des.block<3, 3>(0, 0) = T_right_ct_current.block<3, 3>(0, 0);
    fmt::print("T_left_ct_des: \n{}\nT_right_ct_des: \n{}\n", T_left_ct_des,
               T_right_ct_des);
    // 移动到该位置
    // left
    T_left_ct_current = optitrack.GetTransformcam2target("target_left");
    Eigen::MatrixXd T_left_be_move = GlobalMoveToEndMove(
        T_left_bc, T_left_et, T_left_ct_current, T_left_ct_des);
    Robot::Trajectory left_traj = {{10, T_left_be_move}};
    // right
    T_right_ct_current = optitrack.GetTransformcam2target("target_right");
    Eigen::MatrixXd T_right_be_move = GlobalMoveToEndMove(
        T_right_bc, T_right_et, T_right_ct_current, T_right_ct_des);
    Robot::Trajectory right_traj = {{10, T_right_be_move}};
    // Move
    fmt::print("Left : \nStart at T_left_be_current: \n{}\nEnd at "
               "T_left_be_move: \n{}\n",
               T_left_be_current, T_left_be_move);
    fmt::print("Right : \nStart at T_right_be_current: \n{}\nEnd at "
               "T_right_be_move: \n{}\n",
               T_right_be_current, T_right_be_move);
    robot_left->MovePoseRelative(left_traj, 20ms, 0, 0);
    robot_right->MovePoseRelative(right_traj, 20ms, 0, 0);
    robot_left->startTimer();
    robot_right->startTimer();
    // std::this_thread::sleep_for(11s);
    for (int i = 0; i < 12; i++) {
      std::this_thread::sleep_for(1s);
      fmt::print("\e[1;32mPose=>{}s :\n\e[0m", i + 1);
      fmt::print("Current T_left_ct_current: \n{}\n",
                 optitrack.GetTransformcam2target("target_left"));
      fmt::print("Current T_right_ct_current: \n{}\n",
                 optitrack.GetTransformcam2target("target_right"));
    }
    // 更新Rel TCP, 懒得更新了，TODO 时候再写
    T_left_tcp_rel = T_left_tcp_rel;
    T_right_tcp_rel = T_right_tcp_rel;
    // 当检测到抓紧时跳出循环 ，同上懒得写了
    if (1 /* 检测是否加紧 */) {
      break;
    }
  }
  // #ifdef CONTEXT_MONITOR
  //   server_thread.join();
  // #endif
  // return 0;
  // 下一步，提起锥桶
  fmt::print("Step 2 : Raise the bucket.\n");
  // 为了确保机器人之间保持相对位置关系，需要在se3中进行插值
  robot_left->interpolatePoseInSE3(true);
  robot_right->interpolatePoseInSE3(true);
  // 生成虚拟TCP --
  // 该TCP将同时约束两台机器人，使两台机器人在运动的时候保持固定的相对位置
  Eigen::MatrixXd virtualTCP = optitrack.GetTransformcam2target("object");
  T_left_ct_current = optitrack.GetTransformcam2target("target_left");
  T_right_ct_current = optitrack.GetTransformcam2target("target_right");
  // T_tcp * T_tcp_target = T_target
  Eigen::Matrix4d T_vtcp_target_left = virtualTCP.inverse() * T_left_ct_current;
  Eigen::Matrix4d T_vtcp_target_right =
      virtualTCP.inverse() * T_right_ct_current;
  // Bring up the bucket
  auto VirtualTCPMove = [&](Eigen::MatrixXd virtualTCP_des) {
    auto T_left_ct_des = virtualTCP_des * T_vtcp_target_left;
    auto T_right_ct_des = virtualTCP_des * T_vtcp_target_right;
    T_left_ct_current = optitrack.GetTransformcam2target("target_left");
    T_right_ct_current = optitrack.GetTransformcam2target("target_right");
    auto T_left_be_move = GlobalMoveToEndMove(T_left_bc, T_left_et,
                                              T_left_ct_current, T_left_ct_des);
    auto T_right_be_move = GlobalMoveToEndMove(
        T_right_bc, T_right_et, T_right_ct_current, T_right_ct_des);
    Robot::Trajectory left_traj = {{1, T_left_be_move}};
    Robot::Trajectory right_traj = {{1, T_right_be_move}};
    robot_left->MovePoseRelative(left_traj, 20ms, 0, 0);
    robot_right->MovePoseRelative(right_traj, 20ms, 0, 0);
    robot_left->startTimer();
    robot_right->startTimer();
  };
  while (1) {
    // 固定的TCP
    Eigen::MatrixXd virtualTCP_des = virtualTCP;

    // auto T_left_ct_des = virtualTCP_des * T_vtcp_target_left;
    // auto T_right_ct_des = virtualTCP_des * T_vtcp_target_right;
    // T_left_ct_current = optitrack.GetTransformcam2target("target_left");
    // T_right_ct_current = optitrack.GetTransformcam2target("target_right");
    // auto T_left_be_move = GlobalMoveToEndMove(T_left_bc, T_left_et,
    //                                           T_left_ct_current,
    //                                           T_left_ct_des);
    // auto T_right_be_move = GlobalMoveToEndMove(
    //     T_right_bc, T_right_et, T_right_ct_current, T_right_ct_des);
    // Robot::Trajectory left_traj = {{1, T_left_be_move}};
    // Robot::Trajectory right_traj = {{1, T_right_be_move}};
    // robot_left->MovePoseRelative(left_traj, 20ms, 0, 0);
    // robot_right->MovePoseRelative(right_traj, 20ms, 0, 0);
    // robot_left->startTimer();
    // robot_right->startTimer();
    VirtualTCPMove(virtualTCP_des);
    std::this_thread::sleep_for(1.1s);
    if (1) {
      break;
    }
  }
  // ! 1000s
  T_object_current = optitrack.GetTransformcam2target("object");
  for (int i = 0; i < 10000; i++) {
    // Or 动态的TCP
    std::this_thread::sleep_for(100ms);
    if (optitrack.IsTransformValid("object") == false) {
      fmt::print("\e[1;31mLost the object tracking.\e[0m\n");
      continue;
    }
    T_object_current = optitrack.GetTransformcam2target("object");
    Eigen::MatrixXd virtualTCP_des = T_object_current;
    virtualTCP_des(1, 3) = virtualTCP(1, 3) + 0.1; // 上升0.1m
    VirtualTCPMove(virtualTCP_des);
    if (i % 10 == 0) {
      fmt::print("\e[1;32mPose=>{}s :\n\e[0m", i / 10 + 1);
      fmt::print("Current T_obj: \n{}\n", T_object_current);
      fmt::print("Current T_left_ct: \n{}\n",
                 optitrack.GetTransformcam2target("target_left"));
      fmt::print("Current T_right_ct: \n{}\n",
                 optitrack.GetTransformcam2target("target_right"));
    }
  }
  // gogogo
  // 假设前面步骤已经完成，现在需要针对轴孔装配进行轨迹规划
  fmt::print("Step 3 : Peg in Hole.\n");

#if defined CONTEXT_MONITOR
  server_thread.join();
#endif
}
