#include "optitrack/optitrack.h"
#include <iomanip>
#include <iostream>
OptiTrackRigidBodyCap::OptiTrackRigidBodyCap(
    const std::vector<std::string> &rigid_body_names,
    const std::string &motive_ip) {
  rigidbody_names_ = rigid_body_names;
  motive_ip_ = motive_ip;

  // 初始化容器
  t_cams2targets_.reserve(rigidbody_names_.size());
  Q_cams2targets_.reserve(rigidbody_names_.size());
  T_is_valid_.reserve(rigidbody_names_.size());
  T_mutex_.reserve(rigidbody_names_.size());
  for (int i = 0; i < rigidbody_names_.size(); ++i) {
    t_cams2targets_[rigidbody_names_[i]] = Eigen::Vector3d::Zero();
    Q_cams2targets_[rigidbody_names_[i]] = Eigen::Quaterniond::Identity();
    T_is_valid_[rigidbody_names_[i]] = false;
    T_mutex_[rigidbody_names_[i]] = std::make_unique<std::mutex>();
    //   rigidbody_names_map_[i] = rigidbody_names_[i];
  }
  thread_should_stop_ = 0;
  // 初始化VRPN Tracker
  trackers_.clear();
  for (const auto &rb_name : rigidbody_names_) {
    std::string connection_str = rb_name + "@" + motive_ip_;
    trackers_[rb_name] =
        std::make_shared<vrpn_Tracker_Remote>(connection_str.c_str());
  }
  for (auto &tracker : trackers_) {
    tracker.second->register_change_handler(
        this, &OptiTrackRigidBodyCap::rigidbody_callback);
  }

  cap_thread_ =
      std::make_shared<std::thread>(&OptiTrackRigidBodyCap::CapThread, this);
}
void OptiTrackRigidBodyCap::CapThread() {
  while (!thread_should_stop_) {
    for (auto &[rb_name, tracker] : trackers_) {
      current_tracker_name_ = rb_name;
      tracker->mainloop();
    }
    // ~100Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return;
}
Eigen::Quaterniond
OptiTrackRigidBodyCap::GetQuaternion(const std::string rb_name) {
  std::unique_lock<std::mutex> lock(*T_mutex_[rb_name]);
  return Q_cams2targets_[rb_name];
}
Eigen::MatrixXd
OptiTrackRigidBodyCap::GetTransformcam2target(const std::string rb_name) {
  std::unique_lock<std::mutex> lock(*T_mutex_[rb_name]);
  Eigen::MatrixXd T_cam2target = Eigen::MatrixXd::Identity(4, 4);
  T_cam2target.block<3, 3>(0, 0) = Q_cams2targets_[rb_name].toRotationMatrix();
  T_cam2target.block<3, 1>(0, 3) = t_cams2targets_[rb_name];
  return T_cam2target;
}
bool OptiTrackRigidBodyCap::IsTransformValid(const std::string rb_name) {
  std::unique_lock<std::mutex> lock(*T_mutex_[rb_name]);
  return T_is_valid_[rb_name];
}
OptiTrackRigidBodyCap::~OptiTrackRigidBodyCap() {
  thread_should_stop_ = 1;
  if (cap_thread_ && cap_thread_->joinable()) {
    cap_thread_->join();
  }
  for (auto &[rb_name, tracker] : trackers_) {
    tracker->unregister_change_handler(
        this, &OptiTrackRigidBodyCap::rigidbody_callback);
  }
}
void VRPN_CALLBACK OptiTrackRigidBodyCap::rigidbody_callback(
    void *user_data, const vrpn_TRACKERCB t) {

  OptiTrackRigidBodyCap *self =
      reinterpret_cast<OptiTrackRigidBodyCap *>(user_data);
  // 获取刚体名称（优先使用映射表，其次用原始设备名）
  std::string rb_name = (self->rigidbody_names_map_.count(t.sensor)
                             ? self->rigidbody_names_map_[t.sensor]
                             : self->current_tracker_name_);

  // DEBUG 打印数据（区分不同刚体）
  //
  // std::cout << "\033[1;35m[刚体 " << t.sensor << " (" << rb_name <<
  // ")]\033[0m"
  //           << std::endl;
  // std::cout << "  时间戳: " << t.msg_time.tv_sec << "." << std::setw(6)
  //           << std::setfill('0') << t.msg_time.tv_usec << std::endl;
  // std::cout << "  位置: X=" << std::fixed << std::setprecision(4) << t.pos[0]
  //           << "m, "
  //           << "Y=" << std::fixed << std::setprecision(4) << t.pos[1] << "m,
  //           "
  //           << "Z=" << std::fixed << std::setprecision(4) << t.pos[2] << "m"
  //           << std::endl;
  // std::cout << "  姿态: w=" << std::fixed << std::setprecision(4) <<
  // t.quat[0]
  //           << ", "
  //           << "x=" << std::fixed << std::setprecision(4) << t.quat[1] << ",
  //           "
  //           << "y=" << std::fixed << std::setprecision(4) << t.quat[2] << ",
  //           "
  //           << "z=" << std::fixed << std::setprecision(4) << t.quat[3]
  //           << std::endl;
  // std::cout << "-------------------------" << std::endl;
  Eigen::Quaterniond Q_cam2target(t.quat[3], t.quat[0], t.quat[1], t.quat[2]);
  Eigen::Vector3d t_cam2target(t.pos[0], t.pos[1], t.pos[2]);
  {
    std::unique_lock<std::mutex> lock(*self->T_mutex_[rb_name]);
    self->Q_cams2targets_[rb_name] = Q_cam2target;
    self->t_cams2targets_[rb_name] = t_cam2target;
    self->T_is_valid_[rb_name] = true;
  }
}