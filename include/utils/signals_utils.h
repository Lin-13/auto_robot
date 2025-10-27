#include <Eigen/Core>
#include <condition_variable>
#include <deque>
#include <thread>
#include <vector>
using std::deque;
using std::vector;
namespace SignalFilterType {
const vector<double> smooth_filter_3{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
const vector<double> smooth_filter_phase_3{0.5, 0.25, 0.25};
const vector<double> smooth_filter_5{0.1, 0.2, 0.4, 0.2, 0.1};
const vector<double> smooth_filter_phase_5{3.0 / 8.0, 2.0 / 8.0, 1.0 / 8.0,
                                           1.0 / 8.0, 1.0 / 8.0};
const vector<double> diff1_filter_3{1.0 / 2.0, 0.0, -1.0 / 2.0};
const vector<double> diff1_filter_phase_3{2.0 / 3, -1.0 / 3, -1.0 / 3};
const vector<double> diff1_filter_5{2.0 / 10.0, 1.0 / 10.0, 0, -1.0 / 10.0,
                                    -2.0 / 10.0};
const vector<double> diff1_filter_phase_5{3.0 / 12.0, 2.0 / 12.0, 0,
                                          -1.0 / 12.0, -2.0 / 12.0};
const vector<double> diff2_filter_3{1.0, -2.0, 1.0};
const vector<double> diff2_filter_phase_3{1.0, -2.0, 1.0};
const vector<double> diff2_filter_5{1.0, -4.0, 6.0, -4.0, 1.0};
const vector<double> diff2_filter_phase_5{2.0 / 2.0, -5.0 / 2.0, 4.0 / 2.0, 0,
                                          -1.0 / 2.0};
}; // namespace SignalFilterType
deque<double> signalFilter(const deque<double> &input,
                           const vector<double> &filter);

/**
 * @brief 高阶Butterworth低通滤波器 (BUSF)
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 * @param order 滤波器阶数 (支持1-10阶)
 */
class ButterworthFilter {
public:
  ButterworthFilter(double cutoff_freq, double sample_freq, int order = 2);

  // 滤波单个样本
  double filter(double input);

  // 重置滤波器状态
  void reset();

  // 获取滤波器信息
  int getOrder() const;
  std::vector<double> getCoefficientsA() const;
  std::vector<double> getCoefficientsB() const;

private:
  void calculateCoefficients(double cutoff_freq, double sample_freq, int order);
  void calculateDigitalCoefficients(double normalized_cutoff, int order);

  void
  calculatePolynomialFromRoots(const std::vector<std::complex<double>> &roots,
                               std::vector<double> &coeffs);
  void normalizeCoefficients();

private:
  static const int MAX_ORDER = 10;
  int order_;
  bool initialized_;
  std::vector<double> b_;         // 前向系数 (动态大小)
  std::vector<double> a_;         // 反馈系数 (动态大小)
  std::vector<double> x_history_; // 输入历史 (动态大小)
  std::vector<double> y_history_; // 输出历史 (动态大小)
};

/**
 * @brief 下采样滤波器 - 1000Hz输入，50Hz输出，集成高阶BUSF滤波器
 * @param factor 下采样因子 (默认20，1000Hz/50Hz=20)
 * @param cutoff_freq BUSF滤波器截止频率 (默认25Hz)
 * @param filter_order BUSF滤波器阶数 (默认4阶)
 */
class DownSampleFilter {
public:
  DownSampleFilter(size_t factor = 20, double cutoff_freq = 25.0,
                   int filter_order = 4);

  ~DownSampleFilter();
  // 启动1000Hz数据采集和滤波处理线程
  void start();

  // 停止处理线程
  void stop();

  // 设置数据获取函数（传感器数据源）
  void setPusher(std::function<double()> pusher);

  // 手动推入数据接口
  int highSpeedPush(double value);

  // 获取滤波器信息
  int getFilterOrder() const;

  // 获取最新高阶BUSF滤波值 - 低延迟且高质量滤波
  double getLatestFilteredValue() const;

  // 低速端获取最新滤波数据 (50Hz) - 非阻塞
  bool lowSpeedPop(double &result);

  // 获取最新原始值 - 最低延迟（适用于紧急情况）
  double getLatestRawValue() const;

  // 获取混合值：最新高阶BUSF滤波值 + 下采样趋势
  double getLatestHybridValue();

  // 重置高阶BUSF滤波器
  void resetFilter();

  // 获取缓冲区状态
  size_t getHighSpeedBufferSize() const;

  size_t getLowSpeedBufferSize() const;

  // 获取采样统计信息
  size_t getTotalSampleCount() const;

private:
  // 内部数据处理函数
  int internalPush(double value);
  // 主处理循环：1000Hz数据采集 + 50Hz下采样输出
  void processLoop();

private:
  size_t factor_;                  // 下采样因子
  std::function<double()> pusher_; // 数据推送函数（传感器数据源）
  std::atomic<bool> running_;      // 运行状态
  std::atomic<double> latest_value_;          // 最新原始值
  std::atomic<double> latest_filtered_value_; // 最新高阶BUSF滤波值
  std::atomic<size_t> sample_count_;          // 总采样计数

  ButterworthFilter busf_filter_; // 高阶BUSF滤波器实例

  std::thread processing_thread_;    // 处理线程
  mutable std::mutex mtx_;           // 高速缓冲区互斥锁
  mutable std::mutex low_speed_mtx_; // 低速缓冲区互斥锁
  std::condition_variable cv_;       // 条件变量

  std::deque<double> high_speed_buffer_; // 高速缓冲区(存储高阶BUSF滤波后的值)
  std::deque<double> low_speed_buffer_; // 低速缓冲区(存储下采样值)

  static const size_t HIGH_SPEED_BUFFER_MAX_SIZE = 200;
  static const size_t MAX_LOW_SPEED_HISTORY = 3; // 只保留最近3个下采样值
};