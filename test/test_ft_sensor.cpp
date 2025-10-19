/**
 * @brief Test the FT sensor.
 *  1. Init the FT sensor.
 *  2. Get the measurements.
 *  3. Test the gravity compensation.
 *  在FTSensor 初始化后cin会出现问题，因此使用monitor_server
 * @return int
 */
#include "aubo/aubo_robot.h"
#include "context_monitor/monitor_server.h"
#include "ft_sensor/ft_sensor.h"
#include "utils/matrix_utils.h"
#include <chrono>
#include <filesystem>
#include <fmt/ranges.h>
#include <thread>
using namespace std::chrono_literals;

constexpr char LEFT_ATI_IP[] = "192.168.1.111";
constexpr char RIGHT_ATI_IP[] = "192.168.1.112";
static int signal = 0;
int read_gravity_calib_data(const std::string &filename, Eigen::Vector3d &L,
                            Eigen::Vector3d &G_W, Eigen::Vector3d &f0,
                            Eigen::Vector3d &m0) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    fmt::print("Error: cannot open file {}\n", filename);
    return -1;
  }
  std::string line;
  bool l_read = false;
  bool g_read = false;
  bool f_read = false;
  bool m_read = false;
  while (std::getline(file, line)) {
    // 解析L（格式："L: x y z"）
    if (line.substr(0, 2) == "L:") {
      std::istringstream iss(line.substr(2)); // 提取标签后的内容
      double x, y, z;
      if (iss >> x >> y >> z) { // 解析三个数值
        L = Eigen::Vector3d(x, y, z);
        l_read = true;
      } else {
        // 错误处理：格式不正确
      }
    }
    // 解析G_W（格式："G_W: x y z"）
    else if (line.substr(0, 4) == "G_W:") {
      std::istringstream iss(line.substr(4));
      double x, y, z;
      if (iss >> x >> y >> z) {
        G_W = Eigen::Vector3d(x, y, z);
        g_read = true;
      } else {
        // 错误处理
      }
    }
    // 解析f0（格式："f0: x y z"）
    else if (line.substr(0, 3) == "f0:") {
      std::istringstream iss(line.substr(3));
      double x, y, z;
      if (iss >> x >> y >> z) {
        f0 = Eigen::Vector3d(x, y, z);
        f_read = true;
      } else {
        // 错误处理
      }
    }
    // 解析m0（格式："m0: x y z"）
    else if (line.substr(0, 3) == "m0:") {
      std::istringstream iss(line.substr(3));
      double x, y, z;
      if (iss >> x >> y >> z) {
        m0 = Eigen::Vector3d(x, y, z);
        m_read = true;
      } else {
        // 错误处理
      }
    }
  }
  if (!l_read || !g_read || !f_read || !m_read) {
    fmt::print("Error: cannot read all data from file {}\n", filename);
    return -1;
  }
  return 0;
}
int write_gravity_calib_data(const std::string &filename,
                             const Eigen::Vector3d &L,
                             const Eigen::Vector3d &G_W,
                             const Eigen::Vector3d &f0,
                             const Eigen::Vector3d &m0) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    fmt::print("Error: cannot open file {}\n", filename);
    return -1;
  }
  file << "L:" << L << "\n";
  file << "G_W:" << G_W << "\n";
  file << "f0:" << f0 << "\n";
  file << "m0:" << m0 << "\n";
  return 0;
}
int test_ft_sensor() {
  fmt::print("Init FTSensor left.\n");
  ati::FTSensor left_sensor;
  left_sensor.init(LEFT_ATI_IP);
  std::vector<double> left_measurements(6);
  ati::FTSensor right_sensor;
  right_sensor.init(RIGHT_ATI_IP);
  std::vector<double> right_measurements(6);
  auto start = std::chrono::steady_clock::now();
  while (1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    fmt::print("\033[0;25m{}s\033[0m\n",
               std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start)
                       .count() /
                   1000.0);
    left_sensor.getMeasurements<double>(left_measurements.data());
    fmt::print("left_measurements: {}\n", left_measurements);
    right_sensor.getMeasurements<double>(right_measurements.data());
    fmt::print("right_measurements: {}\n", right_measurements);
  }
  return 0;
}
int gravity_compensation_test(
    const std::string &sensor_name = "left",
    const std::string &data_folder = "gravity_compensation") {
  fmt::print("Init FTSensor {}.\n", sensor_name);
  // 初始化FT传感器
  // 初始化机器人
  ati::FTSensor sensor;
  std::shared_ptr<Robot> robot;
  // if (sensor_name == "left") {
  //   sensor.init(LEFT_ATI_IP);
  //   robot = auboRobotLeft();
  // } else if (sensor_name == "right") {
  //   sensor.init(RIGHT_ATI_IP);
  //   robot = auboRobotRight();
  // } else {
  //   fmt::print("Error: sensor_name must be left or right\n");
  //   return -1;
  // }
  fmt::print("Calibrate start.\n");
  std::vector<double> sensor_measurements(6);
  int cnt = 10;
  std::vector<std::vector<double>> sensor_data;
  std::vector<Eigen::Matrix3d> R_data;
  sensor_data.reserve(cnt);
  for (int i = 0; i < cnt; i++) {
    fmt::print("=== Calibrating {}/{} ===\n", i + 1, cnt);
    fmt::print("Wait signal...\n");
    while (signal == 0) {
      std::this_thread::sleep_for(100ms);
    }
    signal = 0;
    // sensor.getMeasurements<double>(sensor_measurements.data());
    // Eigen::Matrix3d R = robot->currentPose().block<3, 3>(0, 0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R_data.push_back(R);
    fmt::print("{}_measurements: {}\n", sensor_name, sensor_measurements);
    sensor_data.push_back(sensor_measurements);
  }
  fmt::print("Gravity compensation start.\n");
  std::vector<Eigen::Vector3d> F_measure, M_measure;
  for (const auto &data : sensor_data) {
    F_measure.push_back({data[0], data[1], data[2]});
    M_measure.push_back({data[3], data[4], data[5]});
  }
  Eigen::Vector3d L, G_W, f0, m0;
  gravity_compensation(F_measure, M_measure, R_data, L, G_W, f0, m0);
  fmt::print("===Calibration result===\n");
  fmt::print("L: {}\n", L);
  fmt::print("G_W: {}\n", G_W);
  fmt::print("f0: {}\n", f0);
  fmt::print("m0: {}\n", m0);
  // 保存数据
  std::filesystem::create_directories(data_folder);
  std::string filename = fmt::format("{}/{}.txt", data_folder, sensor_name);
  fmt::print("Save data to {}\n", filename);
  write_gravity_calib_data(filename, L, G_W, f0, m0);
  return 0;
}
int main() {
  // test_ft_sensor();
  // test io
  std::thread server_thread(RunMonitorServer, 50051);
  REGISTER_MONITOR_VARIABLE(signal);
  Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();
  REGISTER_MONITOR_VARIABLE(local_transform);

  static Eigen::Vector3d L, G_W, f0, m0;
  L.setOnes();
  REGISTER_MONITOR_VARIABLE(L);
  REGISTER_MONITOR_VARIABLE(G_W);
  REGISTER_MONITOR_VARIABLE(f0);
  REGISTER_MONITOR_VARIABLE(m0);
  std::vector<double> test(3, 3);
  REGISTER_MONITOR_VARIABLE(test[0]);
  gravity_compensation_test("right");
  read_gravity_calib_data("gravity_compensation/right.txt", L, G_W, f0, m0);
  fmt::print("Read data from gravity_compensation/right.txt\n");
  fmt::print("L: {}\n", L);
  fmt::print("G_W: {}\n", G_W);
  fmt::print("f0: {}\n", f0);
  fmt::print("m0: {}\n", m0);

  fmt::print("Gravity compensation test\n");
  fmt::print("Gravity compensation test end\n");
  fmt::print("Press Ctrl+C to stop server\n");
  server_thread.join();
  return 0;
}