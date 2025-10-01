#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <vrpn_Tracker.h>
/**
 * @brief 通过VRPN读取OptiTrack相机的刚体位姿数据
 * @details 通过VRPN读取OptiTrack相机的刚体位姿数据，数据格式为4x4的变换矩阵
 */
// TODO: 读取的数据带有时间戳
class OptiTrackRigidBodyCap {
public:
  OptiTrackRigidBodyCap(const std::vector<std::string> &rigid_body_names,
                        const std::string &motive_ip);
  ~OptiTrackRigidBodyCap();
  Eigen::Quaterniond GetQuaternion(const std::string rb_name);
  Eigen::MatrixXd GetTransformcam2target(const std::string rb_name);
  bool IsTransformValid(const std::string rb_name);

protected:
  static void VRPN_CALLBACK rigidbody_callback(void *user_data,
                                               const vrpn_TRACKERCB t);
  void CapThread();

private:
  // 初始化
  std::vector<std::string> rigidbody_names_;
  std::string motive_ip_;
  std::unordered_map<std::string, std::shared_ptr<vrpn_Tracker_Remote>>
      trackers_;
  [[maybe_unused]] std::unordered_map<int, std::string>
      rigidbody_names_map_; // 传感器ID映射表（传感器ID -> 刚体名称）

  std::atomic<int> thread_should_stop_;
  std::shared_ptr<std::thread> cap_thread_;
  // 容器
  //   std::vector<Eigen::MatrixXd> T_cams2targets;
  std::unordered_map<std::string, Eigen::Quaterniond>
      Q_cams2targets_; // 相机到目标的旋转四元数
  std::unordered_map<std::string, Eigen::VectorXd>
      t_cams2targets_; // 相机到目标的平移向量
  std::unordered_map<std::string, int> T_is_valid_;
  std::unordered_map<std::string, std::unique_ptr<std::mutex>> T_mutex_;
  // 指示线程当前处理的tracker名称
  std::string current_tracker_name_;
};