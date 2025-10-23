#include "aubo/aubo_robot.h"
#include "aubo/aubo_controller.h"
#include "robot_interface/robot.h"
#include "robot_interface/robot_topology.h"
#include "utils/matrix_utils.h"
#include <numbers>

static const char *SERVER_HOST_left = "192.168.1.101";
static const char *SERVER_HOST_right = "192.168.1.131";
static const int SERVER_PORT_left = 8899;
static const int SERVER_PORT_right = 8899;
RobotTopology::Ptr auboRobotTopology() {
  auto robot = std::make_shared<KDL::Chain>();
  // TODO：github上的auboi16参数，与实际参数对不上
  // robot->addSegment(KDL::Segment(
  //     "shoulder_joint", KDL::Joint(KDL::Joint::RotZ),
  //     KDL::Frame(KDL::Rotation::RPY(0, 0, pi), KDL::Vector(0.0, 0.0,
  //     0.163))));
  // robot->addSegment(
  //     KDL::Segment("upperarm_joint", KDL::Joint(KDL::Joint::RotZ),
  //                  KDL::Frame(KDL::Rotation::RPY(-pi / 2, -pi / 2, 0),
  //                             KDL::Vector(0.0, 0.191, 0))));
  // robot->addSegment(KDL::Segment(
  //     "forearm_joint", KDL::Joint(KDL::Joint::RotZ),
  //     KDL::Frame(KDL::Rotation::RPY(-pi, 0, 0), KDL::Vector(0.480, 0, 0))));
  // robot->addSegment(KDL::Segment("wrist1_joint",
  // KDL::Joint(KDL::Joint::RotZ),
  //                                KDL::Frame(KDL::Rotation::RPY(pi, 0, pi /
  //                                2),
  //                                           KDL::Vector(0.36992, 0, 0))));
  // robot->addSegment(KDL::Segment("wrist2_joint",
  // KDL::Joint(KDL::Joint::RotZ),
  //                                KDL::Frame(KDL::Rotation::RPY(-pi / 2, 0,
  //                                0),
  //                                           KDL::Vector(0, 0.1175, 0))));
  // robot->addSegment(KDL::Segment("wrist3_joint",
  // KDL::Joint(KDL::Joint::RotZ),
  //                                KDL::Frame(KDL::Rotation::RPY(pi / 2, 0, 0),
  //                                           KDL::Vector(0, -0.1035, 0))));
  // 硕士论文中的DH参数
  robot->addSegment(KDL::Segment("shoulder_joint", KDL::Joint(KDL::Joint::RotZ),
                                 KDL::Frame::DH(0, pi / 2, 0.163, 0)));
  robot->addSegment(KDL::Segment("upperarm_joint", KDL::Joint(KDL::Joint::RotZ),
                                 KDL::Frame::DH(0.480, -pi, 0.197, pi / 2)));
  robot->addSegment(KDL::Segment("forearm_joint", KDL::Joint(KDL::Joint::RotZ),
                                 KDL::Frame::DH(0.370, -pi, 0.1235, 0)));
  robot->addSegment(KDL::Segment("wrist1_joint", KDL::Joint(KDL::Joint::RotZ),
                                 KDL::Frame::DH(0, -pi / 2, 0.1175, -pi / 2)));
  robot->addSegment(KDL::Segment("wrist2_joint", KDL::Joint(KDL::Joint::RotZ),
                                 KDL::Frame::DH(0, pi / 2, 0.1175, 0)));
  robot->addSegment(KDL::Segment("wrist3_joint", KDL::Joint(KDL::Joint::RotZ),
                                 KDL::Frame::DH(0, 0, 0.1035, 0)));
  // robot->addSegment(KDL::Segment(
  //     "tool0", KDL::Joint(KDL::Joint::Fixed),
  //     KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, 0.1035))));

  RobotTopology::Ptr aubo = std::make_shared<RobotTopology>(*robot);
  KDL::Frame tool_frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, 0));
  aubo->setToolFrame("tool0", tool_frame);
  return aubo;
}
std::shared_ptr<Robot> auboRobotLeft(std::chrono::milliseconds timer_period,
                                     ControllerConfig config) {
  auto robot_topology = auboRobotTopology();
  auto robot_controller = std::make_shared<AuboController>(
      SERVER_HOST_left, SERVER_PORT_left, "aubo", "1", "left_aubo", config);
  int ret = robot_controller->Initialize(timer_period);
  if (ret != 0) {
    std::cerr << "auboRobotLeft : Failed to initialize left robot controller"
              << std::endl;
    return std::make_shared<Robot>(nullptr, robot_topology);
  }
  return std::make_shared<Robot>(robot_controller, robot_topology);
}
std::shared_ptr<Robot> auboRobotRight(std::chrono::milliseconds timer_period,
                                      ControllerConfig config) {
  auto robot_topology = auboRobotTopology();
  auto robot_controller = std::make_shared<AuboController>(
      SERVER_HOST_right, SERVER_PORT_right, "aubo", "1", "right_aubo", config);
  int ret = robot_controller->Initialize(timer_period);
  if (ret != 0) {
    std::cerr << "auboRobotRight : Failed to initialize right robot controller"
              << std::endl;
    return std::make_shared<Robot>(nullptr, robot_topology);
  }
  return std::make_shared<Robot>(robot_controller, robot_topology);
}
