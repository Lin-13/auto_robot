#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>
#include <vrpn_Tracker.h>
// 存储刚体名称与ID的映射
std::unordered_map<int, std::string> rigidbody_names;

// 通用回调函数：处理所有刚体的数据
void VRPN_CALLBACK rigidbody_callback(void *user_data, const vrpn_TRACKERCB t) {
  // 获取刚体名称（优先使用映射表，其次用原始设备名）
  std::string rb_name =
      (rigidbody_names.count(t.sensor) ? rigidbody_names[t.sensor]
                                       : std::string((const char *)user_data));

  // 打印数据（区分不同刚体）
  std::cout << "\033[1;35m[刚体 " << t.sensor << " (" << rb_name << ")]\033[0m"
            << std::endl;
  std::cout << "  时间戳: " << t.msg_time.tv_sec << "." << std::setw(6)
            << std::setfill('0') << t.msg_time.tv_usec << std::endl;
  std::cout << "  位置: X=" << std::fixed << std::setprecision(4) << t.pos[0]
            << "m, "
            << "Y=" << std::fixed << std::setprecision(4) << t.pos[1] << "m, "
            << "Z=" << std::fixed << std::setprecision(4) << t.pos[2] << "m"
            << std::endl;
  std::cout << "  姿态: w=" << std::fixed << std::setprecision(4) << t.quat[0]
            << ", "
            << "x=" << std::fixed << std::setprecision(4) << t.quat[1] << ", "
            << "y=" << std::fixed << std::setprecision(4) << t.quat[2] << ", "
            << "z=" << std::fixed << std::setprecision(4) << t.quat[3]
            << std::endl;
  std::cout << "-------------------------" << std::endl;
}

int main() {
  const std::string motive_ip = "192.168.100.103";
  const std::vector<std::string> rigidbody_list = {"RigidBodyTest"};

  std::vector<std::shared_ptr<vrpn_Tracker_Remote>> trackers;
  for (const auto &rb_name : rigidbody_list) {
    std::string connection_str = rb_name + "@" + motive_ip;
    trackers.push_back(
        std::make_shared<vrpn_Tracker_Remote>(connection_str.c_str()));

    rigidbody_names[trackers.size() - 1] = rb_name;
  }
  for (auto tracker : trackers) {
    tracker->register_change_handler(nullptr, rigidbody_callback);
  }
  std::cout << "开始读取 " << rigidbody_list.size()
            << " 个刚体数据（按Ctrl+C退出）..." << std::endl;
  while (true) {
    for (auto tracker : trackers) {
      tracker->mainloop();
    }

    // 控制更新频率（10ms一次，约100Hz）
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
