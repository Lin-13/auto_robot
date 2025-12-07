#include "aubo/aubo_controller.h"
#include "utils/debug_utils.h"
// #define SERVER_HOST_left "192.168.1.131"
// #define SERVER_PORT_left 8899

// #define SERVER_HOST_right "192.168.1.101"
// #define SERVER_PORT_right 8899
AuboController::AuboController(const char *host_name, int port,
                               const char *user_name, const char *password,
                               std::string name, ControllerConfig config)
    : RobotController(name, 6, config) {
  auto ret_left =
      robot_interface_.robotServiceLogin(host_name, port, user_name, password);
  if (ret_left == aubo::InterfaceCallSuccCode) {
    std::cout << "Aubo Robot login successful." << std::endl;
  } else {
    std::cerr << "Aubo Robot login failed." << std::endl;
  }
}
int AuboController::Initialize(std::chrono::milliseconds timer_period) {
  aubo::ROBOT_SERVICE_STATE result;

  // Tool dynamics parameter
  aubo::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  auto ret = robot_interface_.rootServiceRobotStartup(
      toolDynamicsParam /**Tool dynamics parameter**/, 6 /*Collision level*/,
      true /*Whether to allow reading poses defaults to true*/,
      true,  /*Leave the default to true */
      1000,  /*Leave the default to 1000 */
      result /*Robot arm initialization*/
  );
  if (ret == aubo::InterfaceCallSuccCode) {
    std::cout << "Aubo Robot initialization succeeded." << std::endl;
  } else {
    std::cerr << "Aubo Robot initialization failed." << std::endl;
    return -1;
  }

  robot_interface_.robotServiceInitGlobalMoveProfile();
  // 25->50 deg/s^2
  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  jointMaxAcc.jointPara[0] = 2.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[1] = 2.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[2] = 2.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[3] = 2.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[4] = 2.0 / 180.0 * M_PI;
  jointMaxAcc.jointPara[5] = 2.0 / 180.0 * M_PI;
  // The interface requires the unit to be radians
  robot_interface_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
  jointMaxVelc.jointPara[0] = 5.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[1] = 5.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[2] = 5.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[3] = 5.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[4] = 5.0 / 180.0 * M_PI;
  jointMaxVelc.jointPara[5] = 5.0 / 180.0 * M_PI;
  // The interface requires the unit to be radians
  robot_interface_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

  double lineMoveMaxAcc;
  lineMoveMaxAcc = 0.02; // Units m/s2
  robot_interface_.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
  robot_interface_.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

  double lineMoveMaxVelc;
  lineMoveMaxVelc = 0.02;
  robot_interface_.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
  robot_interface_.robotServiceSetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

  // test
  // std::array<aubo_robot_namespace::JointStatus, joints_> status;
  std::vector<aubo_robot_namespace::JointStatus> status(num_joints_);
  robot_interface_.robotServiceGetRobotJointStatus(status.data(), num_joints_);
  std::vector<double> joint_pos, joint_vel;
  for (auto &sta : status) {
    joint_pos.push_back(sta.jointPosJ);
    joint_vel.push_back(sta.jointSpeedMoto);
  }
  // 允许事实控制
  robot_interface_.robotServiceSetRealTimeJointStatusPush(true);
  // 注册回调函数
  RealTimeJointStatusCallback callback =
      [](const aubo_robot_namespace::JointStatus *jointStatus, int jointNum,
         void *arg) {
        PROFILE_NAME("Aubo RealTimeJointStatusCallback");
        AuboController *controller = static_cast<AuboController *>(arg);
        // do nothing
      };
  robot_interface_.robotServiceSetRealTimeJointStatusPush(true);
  robot_interface_.robotServiceRegisterRealTimeJointStatusCallback(callback,
                                                                   this);
  ret = RobotController::Initialize(timer_period);
  return ret;
}
RobotController::RobotJointState AuboController::getJointState() {
  if (robot_initialized_ == false) {
    std::cerr << "controller not initialized." << std::endl;
    return RobotJointState(Eigen::VectorXd::Zero(num_joints_));
  }
  static bool first_entry = true;
  static std::chrono::time_point<std::chrono::steady_clock> last_time;
  static RobotController::RobotJointState last_joint;
  Eigen::Vector<double, 6> last_vel;
  std::unique_lock<std::mutex> lock(mutex_);
  ////当不是第一次进入时，检查与上次进入的时间间隔，如果小于10ms,则直接返回上次的joint状态
  PROFILE();
  // *
  // if (!first_entry) {
  //   auto now = std::chrono::steady_clock::now();
  //   auto duration =
  //       std::chrono::duration_cast<std::chrono::milliseconds>(now -
  //       last_time)
  //           .count();
  //   if (duration < 10) {
  //     RobotController::RobotJointState new_joint_state;
  //     new_joint_state.joint_state =
  //         last_joint.joint_state + last_vel * (duration / 1000.0);
  //     new_joint_state.timestamp = now;

  //     return new_joint_state;
  //   }
  // }
  std::vector<aubo_robot_namespace::JointStatus> status(num_joints_);
  {
    PROFILE_NAME("robotServiceGetRobotJointStatus");
    // 17ms
    robot_interface_.robotServiceGetRobotJointStatus(status.data(),
                                                     num_joints_);
  }
  std::vector<double> joint_pos, joint_vel;
  for (auto &sta : status) {
    joint_pos.push_back(sta.jointPosJ);
    joint_vel.push_back(sta.jointSpeedMoto);
  };
  RobotJointState joint_state;
  joint_state.joints = num_joints_;
  joint_state.timestamp = std::chrono::steady_clock::now();
  joint_state.joint_state.resize(num_joints_);
  for (int i = 0; i < num_joints_; i++) {
    joint_state.joint_state(i) = joint_pos[i];
    // *
    last_vel[i] = joint_vel[i];
  }
  if (enable_log_ == 1) {
    std::cout << "aubo " << name_ << ": get joint state "
              << joint_state.joint_state.transpose() << std::endl;
  }
  // *
  first_entry = false;
  last_time = joint_state.timestamp;
  last_joint = joint_state;
  return joint_state;
}
int AuboController::setJointState(
    RobotController::RobotJointState joint_state) {
  if (robot_initialized_ == false) {
    std::cerr << "controller not initialized." << std::endl;
    return -1;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  PROFILE();
  if (joint_state.joint_state.size() != num_joints_) {
    std::cerr << "joint state size error." << std::endl;
    return -1;
  }
  // robot_interface_.robotServiceJointMove(joint_state.joint_state.data(),
  // false);
  robot_interface_.robotServiceFollowModeJointMove(
      joint_state.joint_state.data());
  if (enable_log_ == 1) {
    std::cout << "aubo " << name_ << ": set joint state "
              << joint_state.joint_state.transpose() << std::endl;
  }
  return 0;
}
int AuboController::setTarget(RobotJointState target_joint_state) {
  if (enable_log_ == 1) {
    std::cout << "aubo " << name_ << ": set target "
              << target_joint_state.joint_state.transpose() << std::endl;
  }
  RobotController::setTarget(target_joint_state);
  return 0;
}
int AuboController::Logout() {
  if (enable_log_ == 1) {
    std::cout << "Aubo Robot logout." << std::endl;
  }
  robot_interface_.robotServiceLogout();
  return 0;
}
int AuboController::timer_cb() {
  PROFILE_NAME("AuboController::timer_cb");
  static auto start = std::chrono::steady_clock::now();
  auto t_start = std::chrono::steady_clock::now();
  if (enable_log_ == 1) {
    std::cout << "aubo " << name_ << ": timer_cb at "
              << std::chrono::duration_cast<std::chrono::duration<double>>(
                     t_start - start)
                     .count()
              << " s" << std::endl;
  }
  int ret = RobotController::timer_cb();
  if (ret != 0) {
    if (enable_log_ == 1) {
      std::cerr << "aubo " << name_ << ": timer_cb warning, ret: " << ret
                << std::endl;
    }
    return ret;
  }
  auto t_end = std::chrono::steady_clock::now();
  if (enable_log_ == 1) {
    std::cout << "aubo " << name_ << ": timer_cb time run "
              << std::chrono::duration_cast<std::chrono::duration<double>>(
                     t_end - t_start)
                     .count()
              << " s" << std::endl;
  }
  return ret;
}
AuboController::~AuboController() { Logout(); }