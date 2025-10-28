#include <fstream>
#include <utils/debug_utils.h>
#include <utils/signals_utils.h>
void test_gaussian_filter() {
  using namespace SignalFilterType;
  // sigma=10.0，阶数=10
  //   GaussianFilter gaussian_filter(10.0, 11);
  //   GaussianFilter gaussian_filter(25.0, 1000, -1);
  CausalGaussianFilter gaussian_filter(25.0, 1000, -1);
  // 截止频率25Hz，采样频率1000Hz，kernel_size=51

  // 模拟输入信号（例如：正弦波叠加噪声）
  std::vector<double> input_signal;
  std::vector<double> output_signal;
  for (int i = 0; i < 5000; ++i) {
    double t = i / 1000.0;
    double noisy_signal =
        sin(2 * M_PI * 1.0 * t) + 0.5 * ((rand() % 100) / 100.0 - 0.5);
    input_signal.push_back(noisy_signal);
    double filtered_value = gaussian_filter.filter(noisy_signal);
    output_signal.push_back(filtered_value);
  }

  // 输出为文件用于python可视化
  std::ofstream ofs("gaussian_filter_test.csv");
  ofs << "Input,Filtered\n";
  for (size_t i = 0; i < input_signal.size(); ++i) {
    ofs << input_signal[i] << "," << output_signal[i] << "\n";
  }
  ofs.close();
}
void test_butterworth_filter() {
  ButterworthFilter butterworth_filter(25.0, 1000.0, 6); // 提高截止频率到15Hz

  std::vector<double> input_signal;
  std::vector<double> output_signal;

  for (int i = 0; i < 5000; ++i) {
    double t = i / 1000.0; // 时间

    // 组合测试信号：主信号 + 中频噪声 + 高频噪声
    double main_signal = sin(2 * M_PI * 4.0 * t);        // 2Hz主信号
    double mid_noise = 0.25 * sin(2 * M_PI * 50.0 * t);  // 8Hz中频分量
    double high_noise = 0.2 * sin(2 * M_PI * 100.0 * t); // 50Hz高频噪声
    double random_noise = 0.1 * (static_cast<double>(rand()) / RAND_MAX - 0.5);

    double noisy_signal = main_signal + mid_noise + high_noise + random_noise;

    input_signal.push_back(noisy_signal);
    double filtered_value = butterworth_filter.filter(noisy_signal);
    output_signal.push_back(filtered_value);
  }

  // 输出为文件
  std::ofstream ofs("butterworth_filter.csv");
  ofs << "Time,Input,Filtered\n";
  for (size_t i = 0; i < input_signal.size(); ++i) {
    double t = i / 100.0;
    double main_signal = sin(2 * M_PI * 2.0 * t); // 理想信号用于对比
    ofs << t << "," << input_signal[i] << "," << output_signal[i] << "\n";
  }
  ofs.close();

  std::cout << "Optimized Butterworth filter test completed." << std::endl;
}

/**
 * @brief 测试下采样滤波器
 *
 * @return int
 */
int test_downsample_filter() {
  using namespace SignalFilterType;

  // 测试完整的下采样滤波器
  DownSampleFilter downsample_filter(20, 25.0, 2); // 20:1下采样，25Hz截止，2阶

  // 设置1000Hz数据源模拟器
  int sample_count = 0;
  downsample_filter.setPusher([&sample_count]() -> double {
    double t = sample_count / 1000.0; // 1000Hz采样
    sample_count++;

    // 模拟力传感器信号：低频控制信号 + 高频噪声
    double control_signal = 10.0 * sin(2 * M_PI * 3.0 * t); // 3Hz控制信号
    double vibration = 2.0 * sin(2 * M_PI * 60.0 * t);      // 60Hz振动
    double noise = 0.5 * (static_cast<double>(rand()) / RAND_MAX - 0.5);

    return control_signal + vibration + noise;
  });

  downsample_filter.start();

  // 记录50Hz输出数据
  std::ofstream ofs("downsample_filter_test.csv");
  ofs << "Time,RawValue,FilteredValue,DownsampledValue\n";

  auto start_time = std::chrono::steady_clock::now();

  for (int i = 0; i < 100; ++i) { // 记录2秒的50Hz数据
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz

    double raw_value = downsample_filter.getLatestRawValue();
    double filtered_value = downsample_filter.getLatestFilteredValue();
    double downsampled_value;

    if (downsample_filter.lowSpeedPop(downsampled_value)) {
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();

      ofs << elapsed << "," << raw_value << "," << filtered_value << ","
          << downsampled_value << "\n";
    }
  }

  downsample_filter.stop();
  ofs.close();

  std::cout << "Downsample filter test completed." << std::endl;

  return 0;
}

int main() {
  // 测试优化后的butterworth滤波器

  test_butterworth_filter();

  // 测试高斯滤波器
  test_gaussian_filter();

  // 测试完整的下采样滤波器
  //   test_downsample_filter();

  return 0;
}
