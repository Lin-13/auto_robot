#pragma once
#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <memory>
#include <numbers>

static constexpr double pi = std::numbers::pi;
class RobotTopology {
public:
  using Ptr = std::shared_ptr<RobotTopology>;
  RobotTopology(KDL::Chain chain);
  ~RobotTopology();
  Eigen::MatrixXd get_jacob_matrix(const KDL::JntArray &q, int *ret = nullptr);
  Eigen::Matrix4d trans(Eigen::VectorXd q, int *ret = nullptr);
  Eigen::VectorXd trans_inv(Eigen::Matrix4d trans, Eigen::VectorXd q_init,
                            int *ret = nullptr);
  Eigen::VectorXd vel(Eigen::VectorXd q, Eigen::VectorXd q_dot,
                      int *ret = nullptr);
  Eigen::VectorXd vel_inv(Eigen::VectorXd vel, Eigen::VectorXd q_init,
                          int *ret = nullptr);
  int getJointNum();
  KDL::Segment getToolFrame();
  KDL::Segment setToolFrame(std::string name, KDL::Frame);

private:
  std::mutex chain_mutex_;
  KDL::Chain robot_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
};
