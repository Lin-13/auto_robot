#include <gtest/gtest.h>
#include <robot_interface/robot.h>
#include <robot_interface/timer.h>
#include <utils/matrix_utils.h>
class SimpleRobotController : public RobotController {
public:
  SimpleRobotController(std::string name, int num_joints = 6,
                        ControllerConfig config = ControllerConfig(),
                        int print_log = 0)
      : RobotController(name, num_joints, config) {
    sim_state.joint_state = Eigen::VectorXd::Zero(num_joints);
    print_log_ = print_log;
  }
  virtual int
  Initialize(std::chrono::milliseconds timer_period = 33ms) override {
    int ret = RobotController::Initialize(timer_period);
    return ret;
  }
  virtual int setJointState(RobotJointState joint_state) override {
    if (print_log_) {
      std::cerr << "setJointState: " << joint_state.joint_state.transpose()
                << std::endl;
    }
    sim_state = joint_state;
    return 0;
  }
  virtual RobotJointState getJointState() override {
    if (print_log_) {
      std::cerr << "getJointState: " << sim_state.joint_state.transpose()
                << std::endl;
    }
    sim_state.timestamp = std::chrono::steady_clock::now();
    return sim_state;
  }
  int timer_cb() override {
    // // log
    // std::cerr << "Joints: " << getJointState().joint_state.transpose()
    //           << std::endl;
    if (print_log_) {
      std::cerr << "Derived timer_cb" << std::endl;
    }
    RobotController::timer_cb();
    return 0;
  }

public:
  RobotJointState sim_state;
  int print_log_;
};
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

  // frame
  std::cout << "chain joints: " << robot->getNrOfJoints() << std::endl;
  std::cout << "chain segments: " << robot->getNrOfSegments() << std::endl;

  RobotTopology::Ptr aubo = std::make_shared<RobotTopology>(*robot);
  std::cout << "tool0 frame: \n"
            << kdlFrameToEigenXd(aubo->getToolFrame().getFrameToTip())
            << std::endl;
  KDL::Frame tool_frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, 0.2035));
  aubo->setToolFrame("tool0", tool_frame);
  std::cout << "tool0 frame after set: \n"
            << kdlFrameToEigenXd(aubo->getToolFrame().getFrameToTip())
            << std::endl;
  return aubo;
}

void testTimer() {
  auto timer = Timer::create(
      "timer_test",
      []() {
        static int count = 0;
        static auto start_time = std::chrono::steady_clock::now();
        auto time = std::chrono::steady_clock::now();
        std::cout << "timer cb " << count++ << " time : "
                  << std::chrono::duration_cast<std::chrono::microseconds>(
                         time - start_time)
                         .count()
                  << " us" << std::endl;
        return 0;
      },
      100ms);
  std::cout << "start timer" << std::endl;
  auto start_time = std::chrono::steady_clock::now();
  timer->start();
  std::this_thread::sleep_for(2s);
  std::cout << "timer sleep 2s  " << std::endl;
  std::cout << "timer pause" << std::endl;
  timer->pause();
  std::this_thread::sleep_for(1s);
  std::cout << "timer sleep 1s " << std::endl;
  std::cout << "timer resume " << std::endl;
  timer->resume();
  // std::this_thread::sleep_for(1s);
  // std::cout << "timer sleep 1s " << std::endl;
  // std::cout << "timer stop " << std::endl;
  // timer->stop();
}
void testRobotController() {
  auto robot_controller =
      SimpleRobotController("test_robot_controller", 6, {k_p : 2});
  int ret = robot_controller.Initialize();
  if (ret != 0) {
    std::cout << "Robot Controller Initialize failed" << std::endl;
    return;
  }
  RobotController::RobotJointState target;
  target.joint_state = Eigen::VectorXd::Ones(6);
  robot_controller.Run();
  std::cout << "Robot Controller Run" << std::endl;
  // robot_controller.setTarget(target);
  std::cout << "Target: "
            << robot_controller.getTarget().joint_state.transpose()
            << std::endl;
  std::this_thread::sleep_for(3s);
  robot_controller.Stop();
}
void testRobotJointMove() {
  SimpleRobotController::Ptr robot_controller =
      std::make_shared<SimpleRobotController>("test_robot_controller", 6,
                                              ControllerConfig({k_p : 3}));
  int ret = robot_controller->Initialize();
  if (ret != 0) {
    std::cout << "Robot Controller Initialize failed" << std::endl;
    return;
  }
  Robot robot(robot_controller, auboRobotTopology());
  ret = robot.start(30ms);
  if (ret != 0) {
    std::cout << "Robot start failed" << std::endl;
    // return;
  }
  Robot::Trajectory joint_trajectory;
  joint_trajectory.emplace_back(0, Eigen::VectorXd::Zero(6));
  joint_trajectory.emplace_back(3, Eigen::VectorXd::Ones(6));
  joint_trajectory.emplace_back(6, Eigen::VectorXd::Zero(6));
  ret = robot.MoveJoint(joint_trajectory, 500ms);
  if (ret != 0) {
    std::cout << "Trajectory Move failed" << std::endl;
    return;
  }
  std::this_thread::sleep_for(10s);
  robot.stop();
}
void testRobotPoseRelativeMove() {
  auto robot_controller = std::make_shared<SimpleRobotController>(
      "test_robot_controller", 6, ControllerConfig({k_p : 3}));
  int ret = robot_controller->Initialize();

  if (ret != 0) {
    std::cout << "Robot Controller Initialize failed" << std::endl;
    return;
  }

  RobotController::RobotJointState joints;
  joints.joint_state.resize(6);
  joints.joint_state << 50.74, 11.97, -60.18, 108.21, 50.48, 31.67;
  joints.joint_state = joints.joint_state / 180.0 * M_PI;
  robot_controller->sim_state = joints;
  Robot robot(robot_controller, auboRobotTopology());
  if (ret != 0) {
    std::cout << "Robot start failed" << std::endl;
    // return;
  }
  std::cout << "Initial JointState (deg): "
            << robot.currentJointState().transpose() * 180 / M_PI << std::endl;
  std::cout << "Initial Pose: \n" << robot.currentPose() << std::endl;
  Robot::Trajectory pose_trajectory;
  auto T1 =
      HomoMatrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0, -0.02));
  auto T2 =
      HomoMatrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.0, 0, 0.02));
  pose_trajectory.emplace_back(0, Eigen::Matrix4d::Identity());
  pose_trajectory.emplace_back(3, T1);
  pose_trajectory.emplace_back(6, T2);
  ret = robot.start(30ms);
  ret = robot.MovePoseRelative(pose_trajectory, 500ms, 0);
  if (ret != 0) {
    std::cout << "Trajectory Move failed" << std::endl;
    return;
  }
  std::this_thread::sleep_for(8s);
  robot.stop();
  auto result = robot_controller->getJointState();
  std::cout << "Last JointState (deg): "
            << robot.currentJointState().transpose() * 180 / M_PI << std::endl;
  std::cout << "Last Pose: \n" << robot.currentPose() << std::endl;
}
int main() {
  ::testing::InitGoogleTest();
  // std::cout << "testTimer" << std::endl;
  // testTimer();
  // std::cout << "Test Robot Controller" << std::endl;
  // testRobotController();
  // std::cout << "Test Robot Joint Move" << std::endl;
  // testRobotJointMove();
  std::cout << "Test Robot Pose Relative Move" << std::endl;
  testRobotPoseRelativeMove();
  // return RUN_ALL_TESTS();
  return 1;
}
