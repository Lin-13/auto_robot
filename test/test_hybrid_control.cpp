#include "hybrid_control/hybrid_control.h"
#include "utils/matrix_utils.h"
#include "utils/transform_tree.h"
/**
 * @brief 构造transformtree
 *
 *
 * @return TransformTree
 */
TransformTree tree() {
  TransformTree tree;
  tree.add_node("world", Eigen::Matrix4d::Identity(), nullptr, "");
  tree.add_node("origin", Eigen::Matrix4d::Identity(), nullptr, "world");
  tree.add_node("object", Eigen::Matrix4d::Identity(), nullptr, "world");
  // left_robot
  tree.add_node("left_base", Eigen::Matrix4d::Identity(), nullptr, "world");
  tree.add_node("left_end", Eigen::Matrix4d::Identity(), nullptr, "left_base");
  tree.add_node("left_right_body", Eigen::Matrix4d::Identity(), nullptr,
                "left_end");
  tree.add_node("left_tcp", Eigen::Matrix4d::Identity(), nullptr, "left_end");
  tree.add_node("left_ftsensor", Eigen::Matrix4d::Identity(), nullptr,
                "left_end");
  // right_robot
  tree.add_node("right_base", Eigen::Matrix4d::Identity(), nullptr, "world");
  tree.add_node("right_end", Eigen::Matrix4d::Identity(), nullptr,
                "right_base");
  tree.add_node("right_right_body", Eigen::Matrix4d::Identity(), nullptr,
                "right_end");
  tree.add_node("right_tcp", Eigen::Matrix4d::Identity(), nullptr, "right_end");
  tree.add_node("right_ftsensor", Eigen::Matrix4d::Identity(), nullptr,
                "right_end");
  return tree;
}
int main() { HybridController hybrid_control; }
