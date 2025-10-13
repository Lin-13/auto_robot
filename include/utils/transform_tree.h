#include <Eigen/Core>
#include <Eigen/LU>
#include <unordered_map>
using Eigen::Matrix4d;
using Vector6d = Eigen::Vector<double, 6>;
class TransformTree {
public:
  struct TransformNode {
    std::string name;
    Matrix4d transform;
    std::string parent;
    std::vector<std::string> children;
    std::function<Matrix4d()> get_transform_func;
  };
  std::unordered_map<std::string, TransformNode> node_map;
  TransformTree() = default;
  TransformNode &node(const std::string &name) { return node_map[name]; }
  /**
   * @brief 更新所有节点的变换矩阵reset
   *
   */
  void update() {
    for (auto &[name, node] : node_map) {
      if (node.get_transform_func) {
        node.transform = node.get_transform_func();
      }
    }
  }
  void add_node(const std::string &name, const Matrix4d &transform,
                const std::function<Matrix4d()> &get_transform_func,
                const std::string &parent) {
    node_map[name] = {name, transform, parent, {}, get_transform_func};
    if (!parent.empty()) {
      if (!node_exists(parent)) {
        throw std::runtime_error("Parent node not found: " + parent);
      }
      if (std::find(node_map[parent].children.begin(),
                    node_map[parent].children.end(),
                    name) != node_map[parent].children.end()) {
        throw std::runtime_error("Child node already exists: " + name);
      }
      node_map[parent].children.push_back(name);
    }
  }
  void remove_node(const std::string &name) {
    if (!node_exists(name)) {
      throw std::runtime_error("Node not found: " + name);
    }
    // Remove from parent's children list
    const std::string &parent = node_map[name].parent;
    if (!parent.empty() && node_exists(parent)) {
      auto &siblings = node_map[parent].children;
      siblings.erase(std::remove(siblings.begin(), siblings.end(), name),
                     siblings.end());
    }
    // Remove the node itself
    node_map.erase(name);
  }
  int node_exists(const std::string &name) {
    return node_map.find(name) != node_map.end();
  }
  // void add_child(const std::string &parent, const std::string &child) {
  //   if (!node_exists(parent)) {
  //     throw std::runtime_error("Parent node not found: " + parent);
  //   }
  //   if (node_exists(child)) {
  //     throw std::runtime_error("Child node already exists: " + child);
  //   }
  //   node_map[parent].children.push_back(child);
  // }
  // void add_children(const std::string &parent,
  //                   const std::vector<std::string> &children) {
  //   for (const auto &child : children) {
  //     add_child(parent, child);
  //   }
  // }
  void set_transform_func(const std::string &name,
                          const std::function<Matrix4d()> &get_transform_func) {
    if (!node_exists(name)) {
      throw std::runtime_error("Node not found: " + name);
    }
    node_map[name].get_transform_func = get_transform_func;
  }
  Matrix4d get_global_transform(const std::string &name) {
    if (!node_exists(name)) {
      throw std::runtime_error("Node not found: " + name);
    }
    const TransformNode &node = node_map[name];
    Matrix4d local_transform =
        node.get_transform_func ? node.get_transform_func() : node.transform;
    if (node.parent.empty()) {
      return local_transform;
    } else {
      return get_global_transform(node.parent) * local_transform;
    }
  }
  /**
   * @brief 计算两个节点之间的相对变换 - 相对于世界坐标系
   *
   * get_relative_transform_rel和get_relative_transform在参数完全相同时输出一致
   * @param name1 第一个节点的名称
   * @param name2 第二个节点的名称
   * @param view 视图节点的名称，默认值为空字符串-->代表世界坐标系
   * @return Matrix4d 相对变换矩阵
   */
  Matrix4d rel_transform(const std::string &name1, const std::string &name2,
                         const std::string view = "") {
    // 全局坐标系下的相对变换
    Matrix4d relative_transform_in_global =
        get_global_transform(name2) * get_global_transform(name1).inverse();
    Matrix4d relative_transform = relative_transform_in_global;
    if (view != "") {
      if (!node_exists(view)) {
        throw std::runtime_error("View node not found: " + view);
      }
      // 计算view到global的相对变换
      Matrix4d global_to_view = get_global_transform(view);
      // 计算view下从name1到name2的相对变换关系
      relative_transform = global_to_view.inverse() *
                           relative_transform_in_global * global_to_view;
    }
    return relative_transform;
  }
  /**
   * @brief 计算两个节点之间的相对变换 - 相对于视图坐标系
   *
   * get_relative_transform_rel和get_relative_transform在参数完全相同时输出一致
   * @param name1 第一个节点的名称
   * @param name2 第二个节点的名称
   * @param view 视图节点的名称，默认值为空字符串-->代表name1坐标系
   * @return Matrix4d 相对变换矩阵
   */
  Matrix4d rel_transform_rel(const std::string &name1, const std::string &name2,
                             const std::string view = "") {
    // 全局坐标系下的相对变换
    Matrix4d relative_transform_in_global =
        get_global_transform(name1).inverse() * get_global_transform(name2);
    Matrix4d relative_transform = relative_transform_in_global;
    if (view != "") {
      if (!node_exists(view)) {
        throw std::runtime_error("View node not found: " + view);
      }
      // 计算view到name1的相对变换
      Matrix4d global_to_view =
          get_global_transform(view).inverse() * get_global_transform(name1);

      // 计算view下从name1到name2的相对变换关系
      relative_transform = global_to_view * relative_transform_in_global *
                           global_to_view.inverse();
    }
    return relative_transform;
  }
  /**
   * @brief 计算变换矩阵的Adjoint矩阵 T_1 = adjoint(T_0) * twist[w,v]
   *
   * @param T 变换矩阵
   * @return Eigen::Matrix<double, 6, 6> Adjoint矩阵
   */
  Eigen::Matrix<double, 6, 6> adjoint(const Matrix4d &T) {
    Eigen::Matrix<double, 6, 6> Adj = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    Eigen::Matrix3d p_hat;
    p_hat << 0, -p(2), p(1), p(2), 0, -p(0), -p(1), p(0), 0;
    Adj.block<3, 3>(0, 0) = R;
    Adj.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    Adj.block<3, 3>(3, 0) = p_hat * R;
    Adj.block<3, 3>(3, 3) = R;
    return Adj;
  }
  /**
   * @brief 计算力旋量(wrench)的伴随变换矩阵 Ad^*_T
   *        用于力旋量变换: F' = Ad^*_T * F
   *
   * @param T 变换矩阵 SE(3)
   * @return Eigen::Matrix<double, 6, 6> 力旋量的变换矩阵
   */
  Eigen::Matrix<double, 6, 6> adjointStar(const Matrix4d &T) {

    return adjoint(T).transpose();
  }
};
