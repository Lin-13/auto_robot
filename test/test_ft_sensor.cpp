#include "ft_sensor/ft_sensor.h"
#include <fmt/ranges.h>
#include <thread>
constexpr char LEFT_ATI_IP[] = "192.168.1.111";
constexpr char RIGHT_ATI_IP[] = "192.168.1.112";
int test_ft_sensor() {
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
int gravity_compensation_test(const std::string &sensor_name = "left") {
  ati::FTSensor sensor;
  if (sensor_name == "left") {
    sensor.init(LEFT_ATI_IP);
  } else if (sensor_name == "right") {
    sensor.init(RIGHT_ATI_IP);
  } else {
    fmt::print("Error: sensor_name must be left or right\n");
  }
  std::vector<double> sensor_measurements(6);
  int cnt = 10;
  std::vector<std::vector<double>> sensor_data;
  sensor_data.reserve(cnt);
  for (int i = 0; i < cnt; i++) {
    fmt::print("Calibrating... {}/{}\n", i + 1, cnt);
    std::this_thread::sleep_for(std::chrono::seconds(6));
    // auto ch = std::cin.get();
    // fmt::print("ch: {}\n", ch);
    sensor.getMeasurements<double>(sensor_measurements.data());
    fmt::print("{}_measurements: {}\n", sensor_name, sensor_measurements);
    sensor_data.push_back(sensor_measurements);
  }
  return 0;
}
int main() {
  // test_ft_sensor();
  fmt::print("Gravity compensation test\n");
  gravity_compensation_test("right");
  return 0;
}