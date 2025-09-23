#include <robot_interface/robot_topology.h>
// solver ret code
// {
//   E_DEGRADED = +1, E_NOERROR = 0, E_NO_CONVERGE = -1, E_UNDEFINED = -2,
//   E_NOT_UP_TO_DATE = -3, E_SIZE_MISMATCH = -4, E_MAX_ITERATIONS_EXCEEDED =
//   -5, E_OUT_OF_RANGE = -6, E_NOT_IMPLEMENTED = -7, E_SVD_FAILED = -8
// }
/**
 * @brief Construct a new Robot Topology:: Robot Topology object
 * 求解器会绑定chain的引用，因此更改chain后不需要在重新创建新的求解器
 *
 * @param chain
 */
RobotTopology::RobotTopology(KDL::Chain chain)
    : robot_chain_(chain),
      fk_solver_(
          std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_)),
      ik_solver_(std::make_unique<KDL::ChainIkSolverPos_LMA>(robot_chain_)),
      fk_solver_vel_(
          std::make_unique<KDL::ChainFkSolverVel_recursive>(robot_chain_)),
      ik_solver_vel_(
          std::make_unique<KDL::ChainIkSolverVel_pinv>(robot_chain_)),
      jnt_to_jac_solver_(
          std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_)) {}
RobotTopology::~RobotTopology() {}
Eigen::MatrixXd RobotTopology::get_jacob_matrix(const KDL::JntArray &q,
                                                int *ret) {
  KDL::Jacobian jacobian(robot_chain_.getNrOfJoints());
  std::lock_guard<std::mutex> lock(chain_mutex_);
  *ret = jnt_to_jac_solver_->JntToJac(q, jacobian);
  return jacobian.data;
}

Eigen::Matrix4d RobotTopology::trans(Eigen::VectorXd q, int *ret) {
  KDL::Frame frame;
  KDL::JntArray jnt(q.size());
  jnt.data = q;
  {
    std::lock_guard<std::mutex> lock(chain_mutex_);
    int ret_code = fk_solver_->JntToCart(jnt, frame);
    if (ret) {
      *ret = ret_code;
    }
  }
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block<3, 1>(0, 3) << frame.p.x(), frame.p.y(), frame.p.z();
  trans.block<3, 3>(0, 0) << frame.M.data[0], frame.M.data[1], frame.M.data[2],
      frame.M.data[3], frame.M.data[4], frame.M.data[5], frame.M.data[6],
      frame.M.data[7], frame.M.data[8];
  return trans;
}
Eigen::VectorXd RobotTopology::trans_inv(Eigen::Matrix4d trans,
                                         Eigen::VectorXd q_init, int *ret) {
  int num_joints = robot_chain_.getNrOfJoints();
  if (q_init.size() != num_joints) {
    q_init.resize(num_joints);
  }
  KDL::Frame frame;
  frame.p = KDL::Vector(trans(0, 3), trans(1, 3), trans(2, 3));
  frame.M = KDL::Rotation(trans(0, 0), trans(0, 1), trans(0, 2), trans(1, 0),
                          trans(1, 1), trans(1, 2), trans(2, 0), trans(2, 1),
                          trans(2, 2));
  KDL::JntArray jnt_init(num_joints);
  jnt_init.data = q_init;
  {
    std::lock_guard<std::mutex> lock(chain_mutex_);
    int ret_code = ik_solver_->CartToJnt(jnt_init, frame, jnt_init);
    if (ret) {
      *ret = ret_code;
    }
  }
  Eigen::VectorXd q_out(num_joints);
  q_out = jnt_init.data;
  return q_out;
}
/**
 * @brief 计算关节速度
 *
 * @param q 关节角度
 * @param q_dot 关节速度
 * @return Eigen::VectorXd 关节速度
 */
Eigen::VectorXd RobotTopology::vel(Eigen::VectorXd q, Eigen::VectorXd q_dot,
                                   int *ret) {
  if (q.size() != robot_chain_.getNrOfJoints() ||
      q_dot.size() != robot_chain_.getNrOfJoints()) {
    return Eigen::VectorXd::Zero(6);
  }
  KDL::FrameVel frame;

  KDL::JntArrayVel jnt(robot_chain_.getNrOfJoints());
  jnt.q.data = q;
  jnt.qdot.data = q_dot;
  {
    std::lock_guard<std::mutex> lock(chain_mutex_);
    int ret_code = fk_solver_vel_->JntToCart(jnt, frame);
    if (ret) {
      *ret = ret_code;
    }
  }
  Eigen::VectorXd vel(6);
  vel.block<3, 1>(0, 0) =
      Eigen::Map<Eigen::Vector3d>(frame.GetTwist().vel.data, 3);
  vel.block<3, 1>(3, 0) =
      Eigen::Map<Eigen::Vector3d>(frame.GetTwist().rot.data, 3);
  return vel;
}
/**
 * @brief 计算关节速度的逆
 *
 * @param vel 关节速度
 * @param q_init 关节角度初始值
 * @return Eigen::VectorXd 关节速度的逆
 */
Eigen::VectorXd RobotTopology::vel_inv(Eigen::VectorXd vel,
                                       Eigen::VectorXd q_init, int *ret) {
  int num_joints = robot_chain_.getNrOfJoints();
  if (q_init.size() != robot_chain_.getNrOfJoints() || vel.size() != 6) {
    return Eigen::VectorXd::Zero(robot_chain_.getNrOfJoints());
  }
  KDL::JntArray jnt_init(num_joints), jnt_out(num_joints);
  KDL::Twist twist;
  twist.vel[0] = vel(0);
  twist.vel[1] = vel(1);
  twist.vel[2] = vel(2);
  twist.rot[0] = vel(3);
  twist.rot[1] = vel(4);
  twist.rot[2] = vel(5);
  jnt_init.data = q_init;
  {
    std::lock_guard<std::mutex> lock(chain_mutex_);
    int ret_code = ik_solver_vel_->CartToJnt(jnt_init, twist, jnt_out);
    if (ret) {
      *ret = ret_code;
    }
  }
  return jnt_out.data;
}
/**
 * @brief 获取关节数量(去除tool segment)
 *
 * @return int
 */
int RobotTopology::getJointNum() { return robot_chain_.getNrOfJoints(); }

/**
 * @brief 获取tool segment
 * 当robot_chain_为空或tool segment不存在时，返回空segment
 * @return KDL::Segment
 */
KDL::Segment RobotTopology::getToolFrame() {
  if (robot_chain_.getNrOfSegments() == 0 ||
      robot_chain_.getNrOfSegments() == robot_chain_.getNrOfJoints()) {
    return KDL::Segment();
  }
  std::lock_guard<std::mutex> lock(chain_mutex_);
  return robot_chain_.segments.back();
}
/**
 * @brief 设置tool segment
 *
 * @param name tool segment名称
 * @param frame tool segment frame
 * @return KDL::Segment
 */
KDL::Segment RobotTopology::setToolFrame(std::string name, KDL::Frame frame) {
  std::lock_guard<std::mutex> lock(chain_mutex_);
  KDL::Segment tool_segment(name, KDL::Joint(KDL::Joint::Fixed), frame);
  if (robot_chain_.getNrOfSegments() == 0 ||
      robot_chain_.getNrOfSegments() == robot_chain_.getNrOfJoints()) {
    robot_chain_.addSegment(tool_segment);
    return KDL::Segment();
  } else {
    KDL::Segment old_tool_segment =
        robot_chain_.getSegment(robot_chain_.getNrOfSegments() - 1);
    robot_chain_.segments.back() = tool_segment;
    return old_tool_segment;
  }
}
