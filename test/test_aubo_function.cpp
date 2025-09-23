#include <cmath>
#include <iostream>
#include <numbers>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "utils/utils.h"
#include <AuboRobotMetaType.h>
#include <algorithm>
#include <array>
#include <fmt/format.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp> // 位置正解
#include <kdl/chainfksolvervel_recursive.hpp> // 速度正解
#include <kdl/chainiksolverpos_lma.hpp>       // 逆解(LMA算法)
#include <kdl/chainiksolverpos_nr.hpp>        // 数值逆解
#include <kdl/chainiksolvervel_pinv.hpp>      // 速度逆解
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <serviceinterface.h>
#include <string.h>

namespace aubo = aubo_robot_namespace;
#define SERVER_HOST_left "192.168.1.131"
#define SERVER_PORT_left 8899

#define SERVER_HOST_right "192.168.1.101"
#define SERVER_PORT_right 8899

using doublevec = std::vector<double>;
// using namespace std::numbers;
const double pi = 3.14;
int auboi16_robot_topology() {
  KDL::Chain robot;

  robot.addSegment(KDL::Segment(
      "shoulder_joint", KDL::Joint(KDL::Joint::RotZ),
      KDL::Frame(KDL::Rotation::RPY(0, 0, pi), KDL::Vector(0.0, 0.0, 0.163))));
  robot.addSegment(
      KDL::Segment("upperarm_joint", KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame(KDL::Rotation::RPY(-pi / 2, -pi / 2, 0),
                              KDL::Vector(0.0, 0.191, 0))));
  robot.addSegment(KDL::Segment(
      "forearm_joint", KDL::Joint(KDL::Joint::RotZ),
      KDL::Frame(KDL::Rotation::RPY(-pi, 0, 0), KDL::Vector(0.480, 0, 0))));
  robot.addSegment(KDL::Segment("wrist1_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Rotation::RPY(pi, 0, pi / 2),
                                           KDL::Vector(0.36992, 0, 0))));
  robot.addSegment(KDL::Segment("wrist2_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Rotation::RPY(-pi / 2, 0, 0),
                                           KDL::Vector(0, 0.1175, 0))));
  robot.addSegment(KDL::Segment("wrist3_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Rotation::RPY(pi / 2, 0, 0),
                                           KDL::Vector(0, -0.1035, 0))));
  KDL::Segment seg;
  KDL::ChainFkSolverPos_recursive fk_solver(robot);
  KDL::ChainIkSolverPos_LMA ik_solver(robot);
  KDL::ChainFkSolverVel_recursive fk_vel_solver(robot);
  KDL::ChainIkSolverVel_pinv ik_vel_solver(robot);
  KDL::JntArray joint(6);
  joint << doublevec{-24.748530, 29.692813, 94.133893,
                     47.310736,  6.431604,  0.043714};
  std::for_each(joint.data.data(), joint.data.data() + joint.data.size(),
                [](auto &v) { v = v * pi / 180.0; });
  KDL::Frame pose;
  fk_solver.JntToCart(joint, pose);
  Eigen::Map<Eigen::Matrix3d> eigen_map(pose.M.data);
  Eigen::Vector3d rpy1;
  pose.M.GetRPY(rpy1(0), rpy1(1), rpy1(2));
  fmt::print("pose:rpy:{},xyz:{}\n", rpy1, pose.p);
  pose.M;
  // fmt::println("pose:\nRPY:{}\nXYZ:{}",);
  ik_solver.CartToJnt(joint, pose, joint);
  // joint(0) = 1;
  fmt::print("joint:{}\n", joint.data.transpose());
  KDL::JntArrayVel vel(6);
  vel.q << doublevec{0.0, pi / 2, 0, 0, 0, 0};
  vel.qdot << doublevec{0.0, pi / 5, 0, 0, 0, 0};
  fmt::print("Joint & Vel:\n{}\n{}\n", vel.q.data.transpose(),
             vel.qdot.data.transpose());
  KDL::FrameVel pose_vel;
  fk_vel_solver.JntToCart(vel, pose_vel);
  fmt::print("Pose:\n{}\n", pose_vel.GetFrame());
  fmt::print("Vel:\n{}\n", pose_vel.GetTwist());
  return 0;
  // std::cout << robot;
}

int auboi16_robot_topology_dh() {
  KDL::Chain robot;

  robot.addSegment(KDL::Segment("shoulder_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH(0, pi / 2, 0.163, 0)));
  robot.addSegment(KDL::Segment("upperarm_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH(0.480, -pi, 0.197, pi / 2)));
  robot.addSegment(KDL::Segment("forearm_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH(0.370, -pi, 0.1235, 0)));
  robot.addSegment(KDL::Segment("wrist1_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH(0, -pi / 2, 0.1175, -pi / 2)));
  robot.addSegment(KDL::Segment("wrist2_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH(0, pi / 2, 0.1175, 0)));
  robot.addSegment(KDL::Segment("wrist3_joint", KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH(0, 0, 0.1035, 0)));
  KDL::Segment seg;
  KDL::ChainFkSolverPos_recursive fk_solver(robot);
  KDL::ChainIkSolverPos_LMA ik_solver(robot);
  KDL::ChainFkSolverVel_recursive fk_vel_solver(robot);
  KDL::ChainIkSolverVel_pinv ik_vel_solver(robot);
  KDL::JntArray joint(6);
  joint << doublevec{-24.748530, 29.692813, 94.133893,
                     47.310736,  6.431604,  0.043714};
  joint.data = joint.data * pi / 180;
  KDL::Frame pose;
  fk_solver.JntToCart(joint, pose);
  Eigen::Map<Eigen::Matrix3d> eigen_map(pose.M.data);
  Eigen::Vector3d rpy1;
  pose.M.GetRPY(rpy1(0), rpy1(1), rpy1(2));
  fmt::print("pose:rpy:{},xyz:{}\n", rpy1 / pi * 180, pose.p);
  pose.M;
  pose.M = KDL::Rotation::RPY(91.976929 / 180 * pi, 16.975527 / 180 * pi,
                              -18.022593 / 180 * pi);
  pose.p[0] = 0.005682;
  pose.p[1] = -0.326186;
  pose.p[2] = 0.848478;
  ik_solver.CartToJnt(joint, pose, joint);
  // joint(0) = 1;
  fmt::print("joint:{}\n", joint.data.transpose() / pi * 180);
  KDL::JntArrayVel vel(6);
  vel.q << doublevec{0.0, pi / 2, 0, 0, 0, 0};
  vel.qdot << doublevec{0.0, pi / 5, 0, 0, 0, 0};
  fmt::print("Joint & Vel:\n{}\n{}\n", vel.q.data.transpose(),
             vel.qdot.data.transpose());
  KDL::FrameVel pose_vel;
  fk_vel_solver.JntToCart(vel, pose_vel);
  fmt::print("Pose:\n{}\n", pose_vel.GetFrame());
  fmt::print("Vel:\n{}\n", pose_vel.GetTwist());
  return 0;
  // std::cout << robot;
}
void robotInitialize() {
  ServiceInterface robotService_left;
  auto ret_left = robotService_left.robotServiceLogin(
      SERVER_HOST_left, SERVER_PORT_left, "aubo", "1");
  if (ret_left == aubo::InterfaceCallSuccCode) {
    std::cout << "left arm login successful." << std::endl;
  } else {
    std::cerr << "left arm login failed." << std::endl;
  }

  std::cout << "Robot left arm initialization....." << std::endl;

  /** If the real robot arm is connected, the arm needs to be initialized.**/
  aubo::ROBOT_SERVICE_STATE result;

  // Tool dynamics parameter
  aubo::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  ret_left = robotService_left.rootServiceRobotStartup(
      toolDynamicsParam /**Tool dynamics parameter**/, 6 /*Collision level*/,
      true /*Whether to allow reading poses defaults to true*/,
      true,  /*Leave the default to true */
      1000,  /*Leave the default to 1000 */
      result /*Robot arm initialization*/
  );
  if (ret_left == aubo::InterfaceCallSuccCode) {
    std::cout << "Robot left arm initialization succeeded." << std::endl;
  } else {
    std::cerr << "Robot left arm initialization failed." << std::endl;
  }

  robotService_left.robotServiceInitGlobalMoveProfile();

  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  jointMaxAcc.jointPara[0] = 25.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[1] = 25.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[2] = 25.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[3] = 25.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[4] = 25.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[5] =
      25.0 / 180.0 * M_PI; // The interface requires the unit to be radians
  robotService_left.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
  jointMaxVelc.jointPara[0] = 25.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[1] = 25.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[2] = 25.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[3] = 25.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[4] = 25.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[5] =
      25.0 / 180.0 * M_PI; // The interface requires the unit to be radians
  robotService_left.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

  double lineMoveMaxAcc;
  lineMoveMaxAcc = 0.1; // Units m/s2
  robotService_left.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
  robotService_left.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

  double lineMoveMaxVelc;
  lineMoveMaxVelc = 0.1;
  robotService_left.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
  robotService_left.robotServiceSetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

  // read
  std::array<aubo_robot_namespace::JointStatus, 6> status;

  robotService_left.robotServiceGetRobotJointStatus(status.data(), 6);
  std::vector<double> joint_pos, joint_vel;
  for (auto &sta : status) {
    joint_pos.push_back(sta.jointPosJ);
    joint_vel.push_back(sta.jointSpeedMoto);
  }
  aubo::wayPoint_S waypoint;
  //   robotService_left.robotServiceFollowModeJointMove
}

int main() {
  fmt::print("Aubo KDL test function\n");
  auboi16_robot_topology_dh();
  fmt::print("Aubo connect test function\n");
  robotInitialize();
  return 0;
}