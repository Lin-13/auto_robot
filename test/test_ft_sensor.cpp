/**
 * @brief Test the FT sensor.
 *  1. Init the FT sensor.
 *  2. Get the measurements.
 *  3. Test the gravity compensation.
 *  在FTSensor 初始化后cin会出现问题，因此使用monitor_server
 *  v1 test at 2025/10/21 transfer to ft_calib
 */
#include "aubo/aubo_robot.h"
#include "context_monitor/monitor_server.h"
#include "ft_sensor/ft_calib.h"
// 下面实现转移到ft_calib.cpp，删去了函数中的context_monitor
// #include "utils/matrix_utils.h"
// #include "utils/utils.h"
// #include <chrono>
// #include <filesystem>
// #include <fmt/ranges.h>
// #include <thread>
// #include <unordered_map>
// using namespace std::chrono_literals;

// constexpr char LEFT_ATI_IP[] = "192.168.1.111";
// constexpr char RIGHT_ATI_IP[] = "192.168.1.112";

// std::unordered_map<std::string, Eigen::Vector3d>
// read_gravity_calib_data(const std::string &filename) {
//   std::ifstream file(filename);
//   if (!file.is_open()) {
//     fmt::print("Error: cannot open file {}\n", filename);
//     return {};
//   }
//   std::unordered_map<std::string, Eigen::Vector3d> data;
//   std::string line;
//   bool l_read = false;
//   bool g_read = false;
//   bool f_read = false;
//   bool m_read = false;
//   while (std::getline(file, line)) {
//     // 解析L（格式："L: x y z"）
//     if (line.substr(0, 2) == "L:") {
//       std::istringstream iss(line.substr(2)); // 提取标签后的内容
//       double x, y, z;
//       if (iss >> x >> y >> z) { // 解析三个数值
//         data["L"] = Eigen::Vector3d(x, y, z);
//         l_read = true;
//       } else {
//         // 错误处理：格式不正确
//       }
//     }
//     // 解析G_W（格式："G_W: x y z"）
//     else if (line.substr(0, 4) == "G_W:") {
//       std::istringstream iss(line.substr(4));
//       double x, y, z;
//       if (iss >> x >> y >> z) {
//         data["G_W"] = Eigen::Vector3d(x, y, z);
//         g_read = true;
//       } else {
//         // 错误处理
//       }
//     }
//     // 解析f0（格式："f0: x y z"）
//     else if (line.substr(0, 3) == "f0:") {
//       std::istringstream iss(line.substr(3));
//       double x, y, z;
//       if (iss >> x >> y >> z) {
//         data["f0"] = Eigen::Vector3d(x, y, z);
//         f_read = true;
//       } else {
//         // 错误处理
//       }
//     }
//     // 解析m0（格式："m0: x y z"）
//     else if (line.substr(0, 3) == "m0:") {
//       std::istringstream iss(line.substr(3));
//       double x, y, z;
//       if (iss >> x >> y >> z) {
//         data["m0"] = Eigen::Vector3d(x, y, z);
//         m_read = true;
//       } else {
//         // 错误处理
//       }
//     }
//   }
//   if (!l_read || !g_read || !f_read || !m_read) {
//     fmt::print("Error: cannot read all data from file {}\n", filename);
//     return data;
//   }
//   return data;
// }

// int write_gravity_calib_data(
//     const std::string &filename,
//     const std::unordered_map<std::string, Eigen::Vector3d> &params) {
//   std::ofstream file(filename);
//   if (!file.is_open()) {
//     fmt::print("Error: cannot open file {}\n", filename);
//     return -1;
//   }
//   file << "L:" << params.at("L")(0) << " " << params.at("L")(1) << " "
//        << params.at("L")(2) << "\n";
//   file << "G_W:" << params.at("G_W")(0) << " " << params.at("G_W")(1) << " "
//        << params.at("G_W")(2) << "\n";
//   file << "f0:" << params.at("f0")(0) << " " << params.at("f0")(1) << " "
//        << params.at("f0")(2) << "\n";
//   file << "m0:" << params.at("m0")(0) << " " << params.at("m0")(1) << " "
//        << params.at("m0")(2) << "\n";
//   return 0;
// }
// int test_ft_sensor() {
//   fmt::print("Init FTSensor left.\n");
//   ati::FTSensor left_sensor;
//   left_sensor.init(LEFT_ATI_IP);
//   std::vector<double> left_measurements(6);
//   ati::FTSensor right_sensor;
//   right_sensor.init(RIGHT_ATI_IP);
//   std::vector<double> right_measurements(6);
//   auto start = std::chrono::steady_clock::now();
//   while (1) {
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     fmt::print("\033[0;25m{}s\033[0m\n",
//                std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::steady_clock::now() - start)
//                        .count() /
//                    1000.0);
//     left_sensor.getMeasurements<double>(left_measurements.data());
//     fmt::print("left_measurements: {}\n", left_measurements);
//     right_sensor.getMeasurements<double>(right_measurements.data());
//     fmt::print("right_measurements: {}\n", right_measurements);
//   }
//   return 0;
// }
// std::unordered_map<std::string, Eigen::Vector3d>
// test_gravity_compensation(int &signal, std::shared_ptr<ati::FTSensor> sensor,
//                           std::shared_ptr<Robot> robot, int count = 3) {
//   fmt::print("FTSensor Setbias.\n");
//   // 初始化FT传感器
//   // 初始化机器人
//   sensor->setBias();
//   fmt::print("Calibrate start.\n");
//   std::vector<double> sensor_measurements(6);
//   std::vector<std::vector<double>> sensor_data;
//   std::vector<Eigen::Matrix3d> R_data;
//   sensor_data.reserve(count);
//   for (int i = 0; i < count; i++) {
//     fmt::print("=== Calibrating {}/{} ===\n", i + 1, count);
//     fmt::print("Wait signal...\n");
//     while (signal == 0) {
//       std::this_thread::sleep_for(100ms);
//     }
//     signal = 0;
//     sensor->getMeasurements<double>(sensor_measurements.data());
//     Eigen::MatrixXd R = robot->currentPose().block<3, 3>(0, 0);
//     // Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
//     R_data.push_back(R);
//     fmt::print("measurements: {}\n", sensor_measurements);
//     // fmt::print("R: {}\n", R);
//     std::cout << R << std::endl;
//     sensor_data.push_back(sensor_measurements);
//   }
//   fmt::print("Gravity compensation start.\n");
//   std::vector<Eigen::Vector3d> F_measure, M_measure;
//   for (const auto &data : sensor_data) {
//     F_measure.push_back({data[0], data[1], data[2]});
//     M_measure.push_back({data[3], data[4], data[5]});
//   }
//   Eigen::Vector3d L, G_W, f0, m0;
//   gravity_compensation(F_measure, M_measure, R_data, L, G_W, f0, m0);
//   fmt::print("===Calibration result===\n");
//   fmt::print("L: {}\n", L);
//   fmt::print("G_W: {}\n", G_W);
//   fmt::print("f0: {}\n", f0);
//   fmt::print("m0: {}\n", m0);
//   // 保存数据
//   auto params = std::unordered_map<std::string, Eigen::Vector3d>(
//       {{"L", L}, {"G_W", G_W}, {"f0", f0}, {"m0", m0}});

//   // sensor.close();
//   return params;
// }
// Eigen::VectorXd decompose_force(
//     const Eigen::Matrix3d &R, const Eigen::VectorXd &wrench,
//     const std::unordered_map<std::string, Eigen::Vector3d> &params) {
//   if (wrench.size() != 6) {
//     fmt::print("Error: wrench must be 6-dimensional\n");
//     return wrench;
//   }
//   // 分解力
//   Eigen::Vector<double, 6> new_wrench;
//   Eigen::Vector3d G_ = R.transpose() * params.at("G_W");
//   Eigen::Vector3d F_ = wrench.head<3>() - G_ - params.at("f0");
//   Eigen::Vector3d M_ =
//       wrench.tail<3>() - params.at("L").cross(G_) - params.at("m0");
//   new_wrench.head<3>() = F_;
//   new_wrench.tail<3>() = M_;
//   return new_wrench;
// }
// int test_gravity_decompensation(
//     std::shared_ptr<ati::FTSensor> sensor, std::shared_ptr<Robot> robot,
//     const std::unordered_map<std::string, Eigen::Vector3d> &params) {
//   fmt::print("Read data from params.\n");
//   fmt::print("L: {}\n", params.at("L"));
//   fmt::print("G_W: {}\n", params.at("G_W"));
//   fmt::print("f0: {}\n", params.at("f0"));
//   fmt::print("m0: {}\n", params.at("m0"));
//   // 启动传感器
//   auto start = std::chrono::steady_clock::now();
//   Eigen::Vector<double, 6> new_wrench, force_point;
//   new_wrench.setZero();
//   force_point.setZero();
//   REGISTER_MONITOR_VARIABLE(new_wrench);
//   REGISTER_MONITOR_VARIABLE(force_point);
//   while (1) {
//     fmt::print("=====Gravity compensation test=====\n");
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     fmt::print("\033[0;25m{}s\033[0m\n",
//                std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::steady_clock::now() - start)
//                        .count() /
//                    1000.0);
//     Eigen::Vector<double, 6> wrench(6);
//     sensor->getMeasurements<double>(wrench.data());
//     fmt::print("wrench: {}\n", wrench);
//     Eigen::Matrix3d R = robot->currentPose().block<3, 3>(0, 0);
//     new_wrench = decompose_force(R, wrench, params);
//     force_point = new_wrench;
//     force_point.tail<3>() = new_wrench.head<3>().cross(new_wrench.tail<3>())
//     /
//                             new_wrench.head<3>().squaredNorm();
//     fmt::print("new_wrench: {}\n", new_wrench);
//     fmt::print("force_point: {}\n", force_point);
//   }
//   return 0;
// }
//
// ! 标定步骤：
// * 1、先将机器人运动到零位，然后运行test_left,test_gravity_compensation会setbias
// * 2、通过monitor_client将signal置1,此时程序会记录此刻的传感器数据和机器人位姿，重复
// * 3、得到的标定参数会保存到gravity_compensation/{left/right}.txt中
// * 4、需要注意标定参数的f0和m0是在标定时的零位确定的，使用时同样需要将至于相同零位后setBias,此时f0和m0才是正确的
// * 5、标定和使用时要记得R_sensor
// * 6、重力标定和补偿后的力旋量依然不严格等于0,不同位姿其残差不同，原因在于最小二乘的残差
//
void test_left(int &signal) {
  std::shared_ptr<ati::FTSensor> sensor = std::make_shared<ati::FTSensor>();
  sensor->init(LEFT_ATI_IP);
  std::shared_ptr<Robot> robot = auboRobotLeft();
  // calib
  // !!! R_sensor
  Eigen::Matrix3d R_sensor =
      Eigen::AngleAxisd(-165.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  // 重力标定
  sensor->setBias();
  auto params = test_gravity_compensation(signal, sensor, robot, R_sensor, 5);
  write_gravity_calib_data("gravity_compensation/left.txt", params);
  // use calib data
  auto data = read_gravity_calib_data("gravity_compensation/left.txt");
  fmt::print("Read data from gravity_compensation/left.txt\n");
  fmt::print("L: {}\n", data["L"]);
  fmt::print("G_W: {}\n", data["G_W"]);
  fmt::print("f0: {}\n", data["f0"]);
  fmt::print("m0: {}\n", data["m0"]);
  test_gravity_decompensation(sensor, robot, R_sensor, data);
}
void test_right(int &signal) {
  std::shared_ptr<ati::FTSensor> sensor = std::make_shared<ati::FTSensor>();
  sensor->init(RIGHT_ATI_IP);
  std::shared_ptr<Robot> robot = auboRobotRight();
  // calib
  // !!! R_sensor
  Eigen::Matrix3d R_sensor =
      Eigen::AngleAxisd(15.0 * M_PI / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  // 重力标定
  sensor->setBias();
  auto params = test_gravity_compensation(signal, sensor, robot, R_sensor, 5);
  write_gravity_calib_data("gravity_compensation/right.txt", params);
  // use calib data
  auto data = read_gravity_calib_data("gravity_compensation/right.txt");
  fmt::print("Read data from gravity_compensation/right.txt\n");
  fmt::print("L: {}\n", data["L"]);
  fmt::print("G_W: {}\n", data["G_W"]);
  fmt::print("f0: {}\n", data["f0"]);
  fmt::print("m0: {}\n", data["m0"]);
  test_gravity_decompensation(sensor, robot, R_sensor, data);
}
int main() {
  // test_ft_sensor();
  // test io
  std::thread server_thread(RunMonitorServer, 50051);
  static int signal = 0;
  REGISTER_MONITOR_VARIABLE(signal);
  Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();
  REGISTER_MONITOR_VARIABLE(local_transform);
  // init device
  // test_left(signal);
  test_right(signal);
  fmt::print("Press Ctrl+C to stop server\n");
  server_thread.join();
  return 0;
}