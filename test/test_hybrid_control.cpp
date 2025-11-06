#include "aubo/aubo_controller.h"
#include "aubo/aubo_robot.h"
#include "context_monitor/monitor_server.h"
#include "hybrid_control/hybrid_control.h"
#include "optitrack/optitrack.h"
#include "utils/debug_utils.h"
#include "utils/matrix_utils.h"
#include "utils/transform_tree.h"
#include <chrono>
using namespace std::chrono_literals;
/**
 * @brief 构造transformtree
 *
 *
 * @return TransformTree
 */
std::shared_ptr<TransformTree> create_tree() {
  std::shared_ptr<TransformTree> tree = std::make_shared<TransformTree>();
  tree->add_node("world", Eigen::Matrix4d::Identity(), nullptr, "");
  tree->add_node("origin", Eigen::Matrix4d::Identity(), nullptr, "world");
  tree->add_node("object", Eigen::Matrix4d::Identity(), nullptr, "world");
  // left_robot
  // 左机器人基座在optitrack中的位姿
  tree->add_node(
      "left_base",
      readEigenXdFromFile("optitrack_handeye_left/T_bc.txt").inverse(), nullptr,
      "world");
  tree->add_node("left_end", Eigen::Matrix4d::Identity(), nullptr, "left_base");
  // 在optitrack中用于标记左侧末端位姿的刚体
  tree->add_node("left_target",
                 readEigenXdFromFile("optitrack_handeye_left/T_et.txt"),
                 nullptr, "left_end");
  Eigen::Matrix4d left_ftsensor = Eigen::Matrix4d::Identity();
  // * ftsensor相对于末端的固定变换
  left_ftsensor.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(-165.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  left_ftsensor.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.0385);
  tree->add_node("left_ftsensor", left_ftsensor, nullptr, "left_end");
  // * left_tcp相对于ftsensor的固定变换: 绕Z轴旋转15度然后平移142mm
  Eigen::Matrix4d left_tcp = Eigen::Matrix4d::Identity();
  //   left_tcp.block<3, 3>(0, 0) =
  //       Eigen::AngleAxisd(30.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
  //           .toRotationMatrix();
  // 去掉力传感器
  left_tcp.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(-15.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  left_tcp.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.143755);
  //   tree->add_node("left_tcp", left_tcp, nullptr, "left_ftsensor");
  tree->add_node("left_tcp", left_tcp, nullptr, "left_end");
  // ! right_robot
  // * 右机器人基座在optitrack中的位姿
  tree->add_node(
      "right_base",
      readEigenXdFromFile("optitrack_handeye_right/T_bc.txt").inverse(),
      nullptr, "world");
  tree->add_node("right_end", Eigen::Matrix4d::Identity(), nullptr,
                 "right_base");
  // * 在optitrack中用于标记右侧末端位姿的刚体
  tree->add_node("right_target",
                 readEigenXdFromFile("optitrack_handeye_right/T_et.txt"),
                 nullptr, "right_end");
  // * ftsensor相对于末端的固定变换
  Eigen::Matrix4d right_ftsensor = Eigen::Matrix4d::Identity();
  right_ftsensor.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(15.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  right_ftsensor.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.0385);
  tree->add_node("right_ftsensor", right_ftsensor, nullptr, "right_end");
  // * right_tcp相对于ftsensor的固定变换: 绕Z轴旋转15度然后平移142mm
  Eigen::Matrix4d right_tcp = Eigen::Matrix4d::Identity();
  right_tcp.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(30.4 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  right_tcp.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.143755);
  tree->add_node("right_tcp", right_tcp, nullptr, "right_ftsensor");
  return tree;
}
/**
 * @brief 计算标定时的机器人base和实际base的误差
 *          使用构造好的TransformTree和OptiTrackRigidBodyCap获取位姿
 *
 * @param tree
 * @param optitrack
 */
void calib_base_error(std::shared_ptr<TransformTree> tree,
                      std::shared_ptr<OptiTrackRigidBodyCap> optitrack) {
  tree->update();
  auto left_target = tree->get_global_transform("left_target");
  auto right_target = tree->get_global_transform("right_target");
  // 通过optitrack获取的实际刚体位姿反算T_bc[.inv]
  Eigen::Matrix4d left_target_actual =
      optitrack->GetTransformcam2target("target_left");
  Eigen::Matrix4d right_target_actual =
      optitrack->GetTransformcam2target("target_right");
  Eigen::Matrix4d left_target_error =
      left_target_actual.inverse() * left_target;
  std::cout << "Left target estimate error : "
            << left_target_error.block<3, 1>(0, 3).norm() << "\n"
            << "RPY:"
            << RotToRPY(left_target_error.block<3, 3>(0, 0)).transpose() * 180 /
                   M_PI
            << std::endl;
  Eigen::Matrix4d right_target_error =
      right_target_actual.inverse() * right_target;
  std::cout << "Right target estimate error : "
            << right_target_error.block<3, 1>(0, 3).norm() << "\n"
            << "RPY:"
            << RotToRPY(right_target_error.block<3, 3>(0, 0)).transpose() *
                   180 / M_PI
            << std::endl;
  // 计算Tcb的误差
  auto left_bt = tree->rel_transform_rel("left_base", "left_target");
  auto right_bt = tree->rel_transform_rel("right_base", "right_target");
  Eigen::Matrix4d left_base = tree->get_global_transform("left_base");
  Eigen::Matrix4d right_base = tree->get_global_transform("right_base");
  Eigen::Matrix4d left_base_actual = left_target_actual * left_bt.inverse();
  Eigen::Matrix4d right_base_actual = right_target_actual * right_bt.inverse();
  Eigen::Matrix4d left_base_error = left_base_actual.inverse() * left_base;
  Eigen::Matrix4d right_base_error = right_base_actual.inverse() * right_base;
  std::cout << "Left base error : " << left_base_error.block<3, 1>(0, 3).norm()
            << "\n"
            << "RPY:"
            << RotToRPY(left_base_error.block<3, 3>(0, 0)).transpose() * 180 /
                   M_PI
            << std::endl;
  std::cout << "Right base error : "
            << right_base_error.block<3, 1>(0, 3).norm() << "\n"
            << "RPY:"
            << RotToRPY(right_base_error.block<3, 3>(0, 0)).transpose() * 180 /
                   M_PI
            << std::endl;
  return;
}
/**
 * @brief 左右机器人的TCP对齐到夹取姿态
 *
 * @param tree
 * @param optitrack
 * @param left_robo
 * @param right_robo
 */
// T_tcp_new = T_tcp_move * T_tcp_current
// T_tcp_current = T_cb * T_be * T_etcp
// T_tcp_new = T_cb * T_be_new * T_etcp
// => T_tcp_move = T_cb * T_be_new * T_etcp * (T_cb * T_be * T_etcp).inv
// => T_tcp_move = T_cb * T_be_new * T_be.inv * T_cb.inv
// => T_be_new = T_be_new * T_be.inv = T_cb.inv * T_tcp_move * T_cb * T_be
void tcp_pose_init(std::shared_ptr<TransformTree> tree,
                   std::shared_ptr<OptiTrackRigidBodyCap> optitrack,
                   std::shared_ptr<Robot> left_robo,
                   std::shared_ptr<Robot> right_robo) {
  tree->update();
  auto left_end = tree->get_global_transform("left_end");
  auto right_end = tree->get_global_transform("right_end");
  auto left_tcp = tree->get_global_transform("left_tcp");
  auto right_tcp = tree->get_global_transform("right_tcp");
  auto T_cb_left = tree->get_global_transform("left_base");
  auto T_cb_right = tree->get_global_transform("right_base");
  auto T_be_left = tree->rel_transform_rel("left_base", "left_end");
  auto T_be_right = tree->rel_transform_rel("right_base", "right_end");
  auto T_etcp_left = tree->rel_transform_rel("left_end", "left_tcp");
  auto T_etcp_right = tree->rel_transform_rel("right_end", "right_tcp");
  auto left_target = tree->get_global_transform("left_target");
  auto right_target = tree->get_global_transform("right_target");
  Eigen::Matrix4d left_target_actual =
      optitrack->GetTransformcam2target("target_left");
  Eigen::Matrix4d right_target_actual =
      optitrack->GetTransformcam2target("target_right");
  // optitrack下的运动
  // target
  Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d left_tcp_move = I4, right_tcp_move = I4;
  Eigen::Matrix4d left_tcp_target = I4, right_tcp_target = I4;
  left_tcp_target.block<3, 3>(0, 0) << -1, 0, 0, 0, 1, 0, 0, 0, -1;
  right_tcp_target.block<3, 3>(0, 0) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  left_tcp_target.block<3, 1>(0, 3) =
      left_tcp.block<3, 1>(0, 3) + Eigen::Vector3d(0.0, 0.0, 0.0);
  right_tcp_target.block<3, 1>(0, 3) =
      right_tcp.block<3, 1>(0, 3) + Eigen::Vector3d(0.0, 0.0, 0.0);
  // move
  left_tcp_move = left_tcp_target * left_tcp.inverse();
  right_tcp_move = right_tcp_target * right_tcp.inverse();
  Eigen::Matrix4d T_be_move = T_cb_left.inverse() * left_tcp_move * T_cb_left;
  Eigen::Matrix4d T_be_left_new = T_be_move * T_be_left;
  //   T_be_left_new.block<3, 1>(0, 3) = T_be_left.block<3, 1>(0, 3);
  Eigen::Matrix4d T_be_right_new =
      T_cb_right.inverse() * right_tcp_move * T_cb_right * T_be_right;
  //   T_be_right_new.block<3, 1>(0, 3) = T_be_right.block<3, 1>(0, 3);
  auto left_joints = left_robo->currentJointState();
  left_robo->MoveJoint(
      {{0, left_robo->topology()->trans_inv(T_be_left, left_joints)},
       {5, left_robo->topology()->trans_inv(T_be_left_new, left_joints)}},
      30ms, 0, 1);
  auto right_joints = right_robo->currentJointState();
  //   right_robo->MovePose({{0, T_be_right}, {5, T_be_right_new}}, 30ms, 0, 1);
  right_robo->MoveJoint(
      {{0, right_robo->topology()->trans_inv(T_be_right, right_joints)},
       {5, right_robo->topology()->trans_inv(T_be_right_new, right_joints)}},
      30ms, 0, 1);
  std::this_thread::sleep_for(6.5s);
  return;
}
void force_control(std::shared_ptr<TransformTree> tree,
                   std::shared_ptr<Robot> robot) {
  // * 3、 1)力控夹紧
  std::cout << "Force control" << std::endl;
  auto ft_sensor = std::make_shared<ati::FTSensor>();
  ft_sensor->init(RIGHT_ATI_IP);
  auto ft_gravity_compensation = std::make_shared<FTSensorGravityCompensation>(
      ft_sensor, "gravity_compensation/right.txt");
  ft_gravity_compensation->bindPoseDetector([robot]() {
    static Eigen::Matrix3d R_sensor =
        Eigen::AngleAxisd(15.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix3d R = robot->currentPose().block<3, 3>(0, 0);
    R = R * R_sensor;
    return R;
  });
  // * 3、 2)力控
  auto hybrid_control =
      std::make_shared<BaseController3d>(robot, ft_gravity_compensation, tree);
  //   ft_gravity_compensation->setSoftBias();
  ft_gravity_compensation->setSoftBiasMean(500ms);
  // force
  static double x = 0.0, y = 0.0, z = 0.0;
  static Eigen::Vector3d monitor_force;
  static int force_control = 1;
  REGISTER_MONITOR_VARIABLE(x);
  REGISTER_MONITOR_VARIABLE(y);
  REGISTER_MONITOR_VARIABLE(z);
  REGISTER_MONITOR_VARIABLE(monitor_force);
  REGISTER_MONITOR_VARIABLE(force_control);
  for (int i = 0;; i++) {
    PROFILE();
    auto start = std::chrono::steady_clock::now();
    hybrid_control->updateOnce(); // ****
    std::this_thread::sleep_until(start + 20ms);
    // 基坐标系下的force
    monitor_force =
        robot->currentPose().block<3, 3>(0, 0) *
        ft_gravity_compensation->getCompensatedWrench().block<3, 1>(0, 0);
    hybrid_control->setDesiredPos(Eigen::Vector3d(x, y, z));
    if (force_control == 0) {
      break;
    }
    // x = 0.02 * i * 0.01;
    // x = std::clamp(x, -0.05, 0.05);
    // if (monitor_force.x() > -40.0) {
    //   hybrid_control->setDesiredPos(Eigen::Vector3d(x, 0, 0));
    // } else {
    //   hybrid_control->setDesiredForce(Eigen::Vector3d(-40, 0, 0));
    // }
  }
}
void force_control_1d(std::shared_ptr<TransformTree> tree,
                      std::shared_ptr<Robot> robot) {
  // * 3、 1)夹紧检测
  std::cout << "Force control" << std::endl;
  auto ft_sensor = std::make_shared<ati::FTSensor>();
  ft_sensor->init(RIGHT_ATI_IP);
  ft_sensor->setBias();
  std::shared_ptr<FTSensorGravityCompensation> ft_gravity_compensation =
      std::make_shared<FTSensorGravityCompensation>(
          ft_sensor, "gravity_compensation/empty.txt");
  ft_gravity_compensation->bindPoseDetector([robot]() {
    static Eigen::Matrix3d R_sensor =
        Eigen::AngleAxisd(15.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix3d R = robot->currentPose().block<3, 3>(0, 0);
    R = R * R_sensor;
    return R;
  });
  static double x = 0.0, y = 0.0, z = 0.0;
  static Eigen::Vector3d monitor_force;
  static int force_control = 1;
  REGISTER_MONITOR_VARIABLE(x);
  REGISTER_MONITOR_VARIABLE(y);
  REGISTER_MONITOR_VARIABLE(z);
  REGISTER_MONITOR_VARIABLE(monitor_force);
  REGISTER_MONITOR_VARIABLE(force_control);
  static Eigen::Vector<double, 6> ft_measure;
  std::shared_ptr<BaseController> hybrid_control =
      std::make_shared<BaseController>(robot, ft_gravity_compensation, tree, 0);
  ft_gravity_compensation->setSoftBiasMean(500ms);
  for (int i = 0;; i++) {
    PROFILE();
    auto start = std::chrono::steady_clock::now();
    ft_sensor->getMeasurements(ft_measure.data());
    if (force_control == 0) {
      break;
    }
    hybrid_control->updateOnce(); // ****
    // 基坐标系下的force
    monitor_force =
        robot->currentPose().block<3, 3>(0, 0) *
        ft_gravity_compensation->getCompensatedWrench().block<3, 1>(0, 0);
    // monitor_force =
    //     robot->currentPose().block<3, 3>(0, 0) * ft_measure.block<3, 1>(0,
    //     0);
    std::this_thread::sleep_until(start + 20ms);
    // x = 0.02 * i * 0.01;
    // x = std::clamp(x, -0.05, 0.05);
    // if (monitor_force.x() > -40.0) {
    //   hybrid_control->setDesiredPos(Eigen::Vector3d(x, 0, 0));
    // } else {
    //   hybrid_control->setDesiredForce(Eigen::Vector3d(-40, 0, 0));
    // }
  }
}
void dual_move(std::shared_ptr<TransformTree> tree,
               std::shared_ptr<OptiTrackRigidBodyCap> optitrack,
               std::shared_ptr<Robot> left_robo,
               std::shared_ptr<Robot> right_robo) {
  static int dual_move = 1;
  static double virtual_tcp_move_x = 0, virtual_tcp_move_y = 0,
                virtual_tcp_move_z = 0, virtual_tcp_rotate_z = 0,
                virtual_tcp_rotate_y = 0, virtual_tcp_rotate_x = 0;
  static Eigen::Matrix4d target_left =
      optitrack->GetTransformcam2target("target_left");
  static Eigen::Matrix4d target_right =
      optitrack->GetTransformcam2target("target_right");
  Eigen::Matrix4d VirtualTCP = Eigen::Matrix4d::Identity();
  VirtualTCP.block<3, 1>(0, 3) =
      (optitrack->GetTransformcam2target("target_left").block<3, 1>(0, 3) +
       optitrack->GetTransformcam2target("target_right").block<3, 1>(0, 3)) /
      2;
  Eigen::Matrix4d vir2left = VirtualTCP.inverse() * target_left;
  Eigen::Matrix4d vir2right = VirtualTCP.inverse() * target_right;
  int joint_moving = 0, move_start_command = 0;
  double t = 0;
  // monitor
  REGISTER_MONITOR_VARIABLE(t);
  REGISTER_MONITOR_VARIABLE(virtual_tcp_move_x);
  REGISTER_MONITOR_VARIABLE(virtual_tcp_move_y);
  REGISTER_MONITOR_VARIABLE(virtual_tcp_move_z);
  REGISTER_MONITOR_VARIABLE(virtual_tcp_rotate_x);
  REGISTER_MONITOR_VARIABLE(virtual_tcp_rotate_y);
  REGISTER_MONITOR_VARIABLE(virtual_tcp_rotate_z);
  REGISTER_MONITOR_VARIABLE(move_start_command);
  REGISTER_MONITOR_VARIABLE(joint_moving);
  for (int i = 0;; i++) {
    // 更新
    tic();
    tree->update();
    auto T_cb_left = tree->get_global_transform("left_base");
    auto T_cb_right = tree->get_global_transform("right_base");
    auto T_be_left = tree->rel_transform_rel("left_base", "left_end");
    auto T_be_right = tree->rel_transform_rel("right_base", "right_end");
    target_left = optitrack->GetTransformcam2target("target_left");
    target_right = optitrack->GetTransformcam2target("target_right");
    // 计算新的Target
    Eigen::Matrix4d VirtualTCPTarget = VirtualTCP;
    VirtualTCPTarget.block<3, 1>(0, 3) += Eigen::Vector3d(
        virtual_tcp_move_x, virtual_tcp_move_y, virtual_tcp_move_z);
    VirtualTCPTarget.block<3, 3>(0, 0) =
        (Eigen::AngleAxisd(virtual_tcp_rotate_z * M_PI / 180,
                           Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(virtual_tcp_rotate_y * M_PI / 180,
                           Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(virtual_tcp_rotate_x * M_PI / 180,
                           Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    Eigen::Matrix4d target_left_target = VirtualTCPTarget * vir2left;
    Eigen::Matrix4d target_right_target = VirtualTCPTarget * vir2right;
    Eigen::Matrix4d target_left_move =
        target_left_target * target_left.inverse();
    Eigen::Matrix4d target_right_move =
        target_right_target * target_right.inverse();
    Eigen::Matrix4d T_be_left_target =
        T_cb_left.inverse() * target_left_move * T_cb_left * T_be_left;
    Eigen::Matrix4d T_be_right_target =
        T_cb_right.inverse() * target_right_move * T_cb_right * T_be_right;
    // move joint
    auto left_joints = left_robo->currentJointState();
    left_robo->MoveJoint(
        {{0, left_robo->topology()->trans_inv(T_be_left, left_joints)},
         {1, left_robo->topology()->trans_inv(T_be_left_target, left_joints)}},
        30ms, 0, 0);
    auto right_joints = right_robo->currentJointState();
    right_robo->MoveJoint(
        {{0, right_robo->topology()->trans_inv(T_be_right, right_joints)},
         {1,
          right_robo->topology()->trans_inv(T_be_right_target, right_joints)}},
        30ms, 0, 0);
    if (move_start_command == 1) {
      joint_moving = 1;
      left_robo->startTimer();
      right_robo->startTimer();
      move_start_command = 0;
    } else {
      std::cout << "Left Move Pose Relative: " << std::setprecision(3)
                << (T_be_left_target * T_be_left.inverse())
                       .block<3, 1>(0, 3)
                       .transpose()
                << std::endl;
      std::cout << "Right Move Pose Relative: " << std::setprecision(3)
                << (T_be_right_target * T_be_right.inverse())
                       .block<3, 1>(0, 3)
                       .transpose()
                << std::endl;
    }
    std::this_thread::sleep_for(1.1s);
    joint_moving = 0;
    t = toc();
  }
}
void printTrans(Eigen::Matrix4d &trans) {
  std::cout << "Position: " << trans.block<3, 1>(0, 3).transpose() << std::endl;
  std::cout << "RPY: "
            << RotToRPY(trans.block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
}
int main() {
  // 初始化实例
  std::cout << "Initializing instances..." << std::endl;
  std::thread server(RunMonitorServer, 50051);
  auto optitrack = std::make_shared<OptiTrackRigidBodyCap>(
      std::vector<std::string>{"target_left", "target_right"}, "192.168.1.172");
  auto left_robo = auboRobotLeft();
  auto right_robo = auboRobotRight();
  std::shared_ptr<AuboController> left_controller =
      std::dynamic_pointer_cast<AuboController>(left_robo->controller());
  left_controller->enable_log_ = 1;
  std::shared_ptr<AuboController> right_controller =
      std::dynamic_pointer_cast<AuboController>(right_robo->controller());
  right_controller->enable_log_ = 1;
  left_robo->start(30ms);
  right_robo->start(30ms);

  // 设置transform回调
  //   TransformTree tree = create_tree();
  std::shared_ptr<TransformTree> tree = create_tree();
  tree->set_transform_func("left_end",
                           [left_robo]() { return left_robo->currentPose(); });
  tree->set_transform_func(
      "right_end", [right_robo]() { return right_robo->currentPose(); });
  // * 0 测试-无optitrack
  force_control_1d(tree, right_robo);
  return 0;
  // * 1、计算此时的base误差
  // ! 当机器人发生碰撞或者已经夹取过，base的误差会增大
  calib_base_error(tree, optitrack);
  // * 2、基于TCP的运动计算机器人末端的目标位姿
  tcp_pose_init(tree, optitrack, left_robo, right_robo);
  // HybridController hybrid_control;
  // * 3、力控夹紧
  //   force_control(tree, right_robo);
  // * 4、位置控制
  dual_move(tree, optitrack, left_robo, right_robo);
  // 固定左右TCP的相对位置进行移动

  server.join();
}
