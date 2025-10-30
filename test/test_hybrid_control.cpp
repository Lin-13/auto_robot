#include "aubo/aubo_robot.h"
#include "hybrid_control/hybrid_control.h"
#include "optitrack/optitrack.h"
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
TransformTree create_tree() {
  TransformTree tree;
  tree.add_node("world", Eigen::Matrix4d::Identity(), nullptr, "");
  tree.add_node("origin", Eigen::Matrix4d::Identity(), nullptr, "world");
  tree.add_node("object", Eigen::Matrix4d::Identity(), nullptr, "world");
  // left_robot
  // 左机器人基座在optitrack中的位姿
  tree.add_node(
      "left_base",
      readEigenXdFromFile("optitrack_handeye_left/T_bc.txt").inverse(), nullptr,
      "world");
  tree.add_node("left_end", Eigen::Matrix4d::Identity(), nullptr, "left_base");
  // 在optitrack中用于标记左侧末端位姿的刚体
  tree.add_node("left_target",
                readEigenXdFromFile("optitrack_handeye_left/T_et.txt"), nullptr,
                "left_end");
  tree.add_node("left_tcp", Eigen::Matrix4d::Identity(), nullptr, "left_end");
  tree.add_node("left_ftsensor", Eigen::Matrix4d::Identity(), nullptr,
                "left_end");
  // right_robot
  // 右机器人基座在optitrack中的位姿
  tree.add_node(
      "right_base",
      readEigenXdFromFile("optitrack_handeye_right/T_bc.txt").inverse(),
      nullptr, "world");
  tree.add_node("right_end", Eigen::Matrix4d::Identity(), nullptr,
                "right_base");
  // 在optitrack中用于标记右侧末端位姿的刚体
  tree.add_node("right_target",
                readEigenXdFromFile("optitrack_handeye_right/T_et.txt"),
                nullptr, "right_end");
  tree.add_node("right_tcp", Eigen::Matrix4d::Identity(), nullptr, "right_end");
  tree.add_node("right_ftsensor", Eigen::Matrix4d::Identity(), nullptr,
                "right_end");
  return tree;
}
int main() {
  TransformTree tree = create_tree();
  auto left_robo = auboRobotLeft();
  auto right_robo = auboRobotRight();
  OptiTrackRigidBodyCap optitrack({"target_left", "target_right"},
                                  "192.168.1.172");
  // 测试transformtree 能否正确计算
  tree.set_transform_func("left_end",
                          [&left_robo]() { return left_robo->currentPose(); });
  tree.set_transform_func(
      "right_end", [&right_robo]() { return right_robo->currentPose(); });
  for (int i = 0;; i++) {
    std::cout << "=========== Iteration " << i << " ===========" << std::endl;
    tree.update();
    // std::cout
    //     << "Left end in robot: \n"
    //     << left_robo->currentPose() << "\n"
    //     << "RPY:"
    //     << RotToRPY(left_robo->currentPose().block<3, 3>(0, 0)).transpose() *
    //            180 / M_PI
    //     << std::endl;
    // std::cout
    //     << "Right end in robot: \n"
    //     << right_robo->currentPose() << "\n"
    //     << "RPY:"
    //     << RotToRPY(right_robo->currentPose().block<3, 3>(0, 0)).transpose()
    //     *
    //            180 / M_PI
    //     << std::endl;
    auto left_target = tree.get_global_transform("left_target");
    auto right_target = tree.get_global_transform("right_target");
    // std::cout << "Estimate Left target in optitrack: \n"
    //           << left_target << "\n"
    //           << "RPY:"
    //           << RotToRPY(left_target.block<3, 3>(0, 0)).transpose() * 180 /
    //                  M_PI
    //           << std::endl;
    // std::cout << "Estimate Right target in optitrack: \n"
    //           << right_target << "\n"
    //           << "RPY:"
    //           << RotToRPY(right_target.block<3, 3>(0, 0)).transpose() * 180 /
    //                  M_PI
    //           << std::endl;
    // std::cout << "Right target relative to left target in world: \n"
    //           << tree.rel_transform("left_target", "right_target") <<
    //           std::endl;
    // 通过optitrack获取的实际刚体位姿反算T_bc[.inv]
    Eigen::Matrix4d left_target_actual =
        optitrack.GetTransformcam2target("target_left");
    Eigen::Matrix4d right_target_actual =
        optitrack.GetTransformcam2target("target_right");
    // std::cout << "Actual Left target in optitrack: \n"
    //           << left_target_actual << "\n"
    //           << "RPY:"
    //           << RotToRPY(left_target_actual.block<3, 3>(0, 0)).transpose() *
    //                  180 / M_PI
    //           << std::endl;
    // std::cout << "Actual Right target in optitrack: \n"
    //           << right_target_actual << "\n"
    //           << "RPY:"
    //           << RotToRPY(right_target_actual.block<3, 3>(0, 0)).transpose()
    //           *
    //                  180 / M_PI
    //           << std::endl;
    Eigen::Matrix4d left_target_error =
        left_target_actual.inverse() * left_target;
    std::cout << "Left target estimate error : "
              << left_target_error.block<3, 1>(0, 3).norm() << "\n"
              << "RPY:"
              << RotToRPY(left_target_error.block<3, 3>(0, 0)).transpose() *
                     180 / M_PI
              << std::endl;
    Eigen::Matrix4d right_target_error =
        right_target_actual.inverse() * right_target;
    std::cout << "Right target estimate error : "
              << right_target_error.block<3, 1>(0, 3).norm() << "\n"
              << "RPY:"
              << RotToRPY(right_target_error.block<3, 3>(0, 0)).transpose() *
                     180 / M_PI
              << std::endl;
    // 计算新的T_cb
    auto T_cb_left =
        left_target_actual *
        tree.rel_transform_rel("left_base", "left_target").inverse();
    auto T_cb_right =
        right_target_actual *
        tree.rel_transform_rel("right_base", "right_target").inverse();
    // std::cout << "New T_bc_left: \n" << T_cb_left.inverse() << std::endl;
    Eigen::Matrix4d left_base_error =
        T_cb_left.inverse() * tree.get_global_transform("left_base");
    std::cout << "Left base error : "
              << left_base_error.block<3, 1>(0, 3).norm() << "\n"
              << "RPY:"
              << RotToRPY(left_base_error.block<3, 3>(0, 0)).transpose() * 180 /
                     M_PI
              << std::endl;
    // std::cout << "New T_bc_right: \n" << T_cb_right.inverse() << std::endl;
    Eigen::Matrix4d right_base_error =
        T_cb_right.inverse() * tree.get_global_transform("right_base");
    std::cout << "Right base error : "
              << right_base_error.block<3, 1>(0, 3).norm() << "\n"
              << "RPY:"
              << RotToRPY(right_base_error.block<3, 3>(0, 0)).transpose() *
                     180 / M_PI
              << std::endl;
    std::this_thread::sleep_for(1s);
  }
  // HybridController hybrid_control;
}
