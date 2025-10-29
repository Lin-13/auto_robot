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
 * @brief 级联滤波器结构体
 *
 */
struct CascadeSection {
  double b0, b1, b2, a1, a2;
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

  double filter(double input) {
    double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    // 更新延迟
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;

    return output;
  }
};
class FilterBase {
public:
  virtual double filter(double input) = 0;
  virtual void reset() = 0;
  virtual ~FilterBase() = default;
};
class GaussianFilter : public FilterBase {
public:
  // 构造函数1：通过sigma和kernel_size
  GaussianFilter(double sigma, int kernel_size)
      : sigma_(sigma), kernel_size_(kernel_size) {
    if (kernel_size < 3 || kernel_size % 2 == 0) {
      throw std::invalid_argument("Kernel size must be odd and >= 3");
    }
    cutoff_freq_ = calculateCutoffFromSigma(sigma);
    generateKernel();
  }

  // 构造函数2：通过截止频率和采样频率
  GaussianFilter(double cutoff_freq, double sample_freq, int kernel_size) {
    cutoff_freq_ = cutoff_freq;
    sample_freq_ = sample_freq;

    // 从截止频率计算sigma（数字域）
    double normalized_cutoff = cutoff_freq / (sample_freq / 2.0);
    sigma_ = calculateSigmaFromNormalizedCutoff(normalized_cutoff);

    // 自动确定kernel大小
    if (kernel_size == -1) {
      kernel_size_ = calculateOptimalKernelSize(sigma_);
    } else {
      kernel_size_ = kernel_size;
    }

    generateKernel();
  }

  // 获取滤波器参数
  double getSigma() const { return sigma_; }
  double getCutoffFreq() const { return cutoff_freq_; }
  int getKernelSize() const { return kernel_size_; }
  std::vector<double> getKernel() const { return kernel_; }

  double filter(double input) override {
    // 使用卷积计算滤波
    if (!initialized_) {
      buffer_ = std::vector<double>(kernel_size_, input);
      initialized_ = true;
      return input;
    }

    // 更新缓冲区
    buffer_.erase(buffer_.begin());
    buffer_.push_back(input);

    // 卷积计算
    double output = 0.0;
    for (size_t i = 0; i < kernel_.size(); ++i) {
      output += kernel_[i] * buffer_[i];
    }
    return output;
  }

  void reset() override {
    buffer_.clear();
    initialized_ = false;
  }

private:
  void generateKernel() {
    kernel_.resize(kernel_size_);
    double sum = 0.0;
    int center = kernel_size_ / 2;

    // 生成高斯核
    for (int i = 0; i < kernel_size_; ++i) {
      double x = i - center;
      kernel_[i] = std::exp(-(x * x) / (2.0 * sigma_ * sigma_));
      sum += kernel_[i];
    }

    // 归一化
    for (double &value : kernel_) {
      value /= sum;
    }
  }

  double calculateCutoffFromSigma(double sigma) {
    return 0.133 / sigma; // -3dB截止频率
  }

  double calculateSigmaFromNormalizedCutoff(double normalized_cutoff) {
    // 对于数字高斯滤波器的经验公式
    return 0.133 / normalized_cutoff;
  }

  int calculateOptimalKernelSize(double sigma) {
    // 经验法则：kernel大小约为6σ
    int size = static_cast<int>(6.0 * sigma + 1);
    return (size % 2 == 0) ? size + 1 : size; // 确保为奇数
  }

private:
  double sigma_;
  double cutoff_freq_;
  double sample_freq_;
  int kernel_size_;
  std::vector<double> kernel_;
  std::vector<double> buffer_;
  bool initialized_ = false;
};

/**
 * @brief 因果高斯滤波器 (CGF)
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 * @param order 级联阶数 (默认2阶)
 */
class CausalGaussianFilter : public FilterBase {
public:
  CausalGaussianFilter(double cutoff_freq, double sample_freq, int order = -1)
      : cutoff_freq_(cutoff_freq), sample_freq_(sample_freq) {

    double normalized_cutoff = cutoff_freq / (sample_freq / 2.0);

    // 计算等效的时间常数
    tau_ = 1.0 / (2.0 * M_PI * cutoff_freq); // 时间常数

    // 转换为数字域的衰减因子
    double dt = 1.0 / sample_freq;
    alpha_ = dt / (tau_ + dt); // 一阶低通滤波器系数

    // 如果需要更高阶的近似，可以级联多个一阶段
    if (order == -1) {
      order_ = 2; // 默认2阶
    } else {
      order_ = order;
    }

    // 初始化级联段
    stages_.resize(order_, 0.0);
  }

  double filter(double input) override {
    double output = input;

    // 级联多个一阶低通滤波器来近似高斯特性
    for (int i = 0; i < order_; ++i) {
      stages_[i] = alpha_ * output + (1.0 - alpha_) * stages_[i];
      output = stages_[i];
    }

    return output;
  }

  void reset() override { std::fill(stages_.begin(), stages_.end(), 0.0); }

  double getGroupDelay() const {
    // 级联一阶滤波器的群延迟
    return order_ * tau_;
  }

private:
  double cutoff_freq_;
  double sample_freq_;
  double tau_;                 // 时间常数
  double alpha_;               // 数字滤波器系数
  int order_;                  // 级联阶数
  std::vector<double> stages_; // 级联段状态
};
/**
 * @brief 高阶Butterworth低通滤波器 (BUSF)
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 * @param order 滤波器阶数 (支持1-10阶)
 */
class ButterworthFilter : public FilterBase {
public:
  ButterworthFilter(double cutoff_freq, double sample_freq, int order = 2);

  // 滤波单个样本
  double filter(double input) override;

  // 重置滤波器状态
  void reset() override;

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
  void calculateHighOrderCoefficients(double normalized_cutoff, int order);
  void checkStability();

private:
  static const int MAX_ORDER = 10;
  int order_;
  std::vector<CascadeSection> cascaded_sections_;
  bool initialized_;
  std::vector<double> b_;         // 前向系数 (动态大小)
  std::vector<double> a_;         // 反馈系数 (动态大小)
  std::vector<double> x_history_; // 输入历史 (动态大小)
  std::vector<double> y_history_; // 输出历史 (动态大小)
};

/**
 * @brief 下采样滤波器
 * 该类以指定的下采样因子对输入信号进行下采样处理，并可选地应用低通滤波器以减少混叠。
 * 该类以sample_freq频率调用pusher
 * @param factor 下采样因子
 * @param sample_freq 输入采样频率 (默认1000Hz)
 * @param cutoff_freq 滤波器截止频率 (默认25Hz)
 * @param filter_order 滤波器阶数 (默认4阶)
 */
class DownSampleFilter {
public:
  DownSampleFilter(int factor, double sample_freq = 1000.0,
                   double cutoff_freq = 25.0, int filter_order = 4);

  ~DownSampleFilter();
  // 启动1000Hz数据采集和滤波处理线程
  void start();

  // 停止处理线程
  void stop();

  // 设置数据获取函数（传感器数据源）
  void setPusher(std::function<double()> pusher);

  // 手动推入数据接口
  int highSpeedPush(double value);

  // 获取最新滤波值 - 低延迟且高质量滤波
  double getLatestFilteredValue() const;

  // 低速端获取最新滤波数据 (50Hz) - 非阻塞
  bool lowSpeedPop(double &result);

  // 获取最新原始值 - 最低延迟（适用于紧急情况）
  double getLatestRawValue() const;

  // 获取混合值：最新滤波值 + 下采样趋势
  double getLatestHybridValue();

  // 重置滤波器
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
  double sample_freq_;             // 输入采样频率
  double cutoff_freq_;             // 滤波器截止频率
  std::function<double()> pusher_; // 数据推送函数（传感器数据源）
  std::atomic<bool> running_;      // 运行状态
  std::atomic<double> latest_value_;          // 最新原始值
  std::atomic<double> latest_filtered_value_; // 最新滤波值
  std::atomic<size_t> sample_count_;          // 总采样计数

  std::shared_ptr<FilterBase> filter_; // 滤波器实例

  std::thread processing_thread_;    // 处理线程
  mutable std::mutex mtx_;           // 高速缓冲区互斥锁
  mutable std::mutex low_speed_mtx_; // 低速缓冲区互斥锁
  std::condition_variable cv_;       // 条件变量

  std::deque<double> high_speed_buffer_; // 高速缓冲区(存储高阶BUSF滤波后的值)
  std::deque<double> low_speed_buffer_; // 低速缓冲区(存储下采样值)

  static const size_t HIGH_SPEED_BUFFER_MAX_SIZE = 200;
  static const size_t MAX_LOW_SPEED_HISTORY = 3; // 只保留最近3个下采样值
};
