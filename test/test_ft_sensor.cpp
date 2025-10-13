#include "ft_sensor/ft_sensor.h"
#include <fmt/ranges.h>
int main() {
  ati::FTSensor sensor;
  sensor.init("ip");
  std::vector<double> measurements(6);
  sensor.getMeasurements<double>(measurements.data());
  fmt::print("measurements: {}\n", measurements);
  return 0;
}
