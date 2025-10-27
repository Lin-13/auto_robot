#include "utils/signals_utils.h"
#include <atomic>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
using std::deque;
/**
 * @brief 微分信号滤波器
 * @param input 输入信号
 * @param filter 滤波器系数
 * @return deque<double> 输出信号
 */
deque<double> signalFilter(const deque<double> &input,
                           const vector<double> &filter) {
  if (filter.size() >= input.size()) {
    throw std::invalid_argument("filter size must be less than input size");
  }
  deque<double> output;
  for (size_t i = 0; i < filter.size(); i++) {
    output.push_back(0.0);
  }
  const size_t filter_size = filter.size();
  for (size_t i = filter_size; i < input.size(); i++) {
    double sum = 0.0;
    for (size_t j = 0; j < filter_size; j++) {
      sum += input[i - j] * filter[j];
    }
    output.push_back(sum);
  }
  return output;
}
/******************************ButterworthFilter*********************************/
// 滤波单个样本
ButterworthFilter::ButterworthFilter(double cutoff_freq, double sample_freq,
                                     int order)
    : order_(order), initialized_(false) {
  if (order < 1 || order > MAX_ORDER) {
    throw std::invalid_argument("Filter order must be between 1 and " +
                                std::to_string(MAX_ORDER));
  }

  // 动态分配系数和历史数组
  b_.resize(order_ + 1, 0.0);
  a_.resize(order_ + 1, 0.0);
  x_history_.resize(order_ + 1, 0.0);
  y_history_.resize(order_ + 1, 0.0);

  calculateCoefficients(cutoff_freq, sample_freq, order);
  reset();
}
double ButterworthFilter::filter(double input) {
  if (!initialized_) {
    // 初始化历史值为第一个输入
    for (int i = 0; i <= order_; i++) {
      x_history_[i] = input;
      y_history_[i] = input;
    }
    initialized_ = true;
    return input;
  }

  // 移位历史数组
  for (int i = order_; i > 0; i--) {
    x_history_[i] = x_history_[i - 1];
    y_history_[i] = y_history_[i - 1];
  }
  x_history_[0] = input;

  // 计算输出
  double output = 0.0;
  for (int i = 0; i <= order_; i++) {
    output += b_[i] * x_history_[i];
  }
  for (int i = 1; i <= order_; i++) {
    output -= a_[i] * y_history_[i];
  }

  y_history_[0] = output;
  return output;
}

// 重置滤波器状态
void ButterworthFilter::reset() {
  std::fill(x_history_.begin(), x_history_.end(), 0.0);
  std::fill(y_history_.begin(), y_history_.end(), 0.0);
  initialized_ = false;
}

// 获取滤波器信息
int ButterworthFilter::getOrder() const { return order_; }
std::vector<double> ButterworthFilter::getCoefficientsA() const { return a_; }
std::vector<double> ButterworthFilter::getCoefficientsB() const { return b_; }

void ButterworthFilter::calculateCoefficients(double cutoff_freq,
                                              double sample_freq, int order) {
  double nyquist = sample_freq / 2.0;
  double normalized_cutoff = cutoff_freq / nyquist;

  if (normalized_cutoff >= 1.0) {
    throw std::invalid_argument(
        "Cutoff frequency must be less than Nyquist frequency");
  }

  // 使用双线性变换计算数字滤波器系数
  calculateDigitalCoefficients(normalized_cutoff, order);
}

void ButterworthFilter::calculateDigitalCoefficients(double normalized_cutoff,
                                                     int order) {
  // 计算模拟滤波器极点
  std::vector<std::complex<double>> poles;
  for (int k = 0; k < order; k++) {
    double theta = M_PI * (2 * k + order + 1) / (2 * order);
    poles.push_back(std::complex<double>(cos(theta), sin(theta)));
  }

  // 预扭曲频率
  double wc = tan(M_PI * normalized_cutoff);

  // 将极点从单位圆映射到左半平面，并进行频率缩放
  for (auto &pole : poles) {
    pole *= wc;
  }

  // 使用双线性变换将s域极点转换为z域
  std::vector<std::complex<double>> z_poles;
  for (const auto &s_pole : poles) {
    std::complex<double> z_pole = (1.0 + s_pole / 2.0) / (1.0 - s_pole / 2.0);
    z_poles.push_back(z_pole);
  }

  // 计算分母多项式系数 (从极点)
  calculatePolynomialFromRoots(z_poles, a_);

  // 计算分子多项式系数 (全部为1，低通特性)
  std::fill(b_.begin(), b_.end(), 0.0);
  for (int i = 0; i <= order; i++) {
    b_[i] = 1.0;
  }

  // 归一化系数
  normalizeCoefficients();
}

void ButterworthFilter::calculatePolynomialFromRoots(
    const std::vector<std::complex<double>> &roots,
    std::vector<double> &coeffs) {
  coeffs.assign(order_ + 1, 0.0);
  coeffs[0] = 1.0;

  for (const auto &root : roots) {
    // 乘以 (z - root)
    for (int i = order_; i >= 1; i--) {
      coeffs[i] = coeffs[i - 1] - root.real() * coeffs[i];
      if (std::abs(root.imag()) > 1e-10) {
        // 处理复数根 - 必须成对出现
        coeffs[i] += std::norm(root) * coeffs[i];
      }
    }
    coeffs[0] *= -root.real();
  }
}

void ButterworthFilter::normalizeCoefficients() {
  // 计算直流增益
  double dc_gain_num = 0.0, dc_gain_den = 0.0;
  for (int i = 0; i <= order_; i++) {
    dc_gain_num += b_[i];
    dc_gain_den += a_[i];
  }

  // 归一化分母系数
  double a0 = a_[0];
  for (int i = 0; i <= order_; i++) {
    a_[i] /= a0;
    b_[i] /= a0;
  }

  // 调整分子系数以获得单位直流增益
  double gain_correction = dc_gain_den / dc_gain_num;
  for (int i = 0; i <= order_; i++) {
    b_[i] *= gain_correction;
  }
}
/**
 * @brief 高阶Butterworth低通滤波器 (BUSF)
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 * @param order 滤波器阶数 (支持1-10阶)
 */
DownSampleFilter::DownSampleFilter(size_t factor, double cutoff_freq,
                                   int filter_order)
    : factor_(factor), running_(false), sample_count_(0),
      busf_filter_(cutoff_freq, 1000.0, filter_order) {
  latest_value_.store(0.0);
  latest_filtered_value_.store(0.0);
}

DownSampleFilter::~DownSampleFilter() { stop(); }

// 启动1000Hz数据采集和滤波处理线程
void DownSampleFilter::start() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!running_) {
    running_ = true;
    processing_thread_ = std::thread(&DownSampleFilter::processLoop, this);
  }
}

// 停止处理线程
void DownSampleFilter::stop() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    running_ = false;
  }
  cv_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

// 设置数据获取函数（传感器数据源）
void DownSampleFilter::setPusher(std::function<double()> pusher) {
  std::lock_guard<std::mutex> lock(mtx_);
  pusher_ = pusher;
}

// 手动推入数据接口
int DownSampleFilter::highSpeedPush(double value) {
  return internalPush(value);
}

// 获取滤波器信息
int DownSampleFilter::getFilterOrder() const { return busf_filter_.getOrder(); }

// 获取最新高阶BUSF滤波值 - 低延迟且高质量滤波
double DownSampleFilter::getLatestFilteredValue() const {
  return latest_filtered_value_.load();
}

// 低速端获取最新滤波数据
bool DownSampleFilter::lowSpeedPop(double &result) {
  std::lock_guard<std::mutex> lock(low_speed_mtx_);
  if (low_speed_buffer_.empty()) {
    return false;
  }
  result = low_speed_buffer_.back(); // 获取最新的下采样值
  low_speed_buffer_.clear();         // 清空缓冲区，避免累积延迟
  return true;
}

// 获取最新原始值 - 最低延迟
double DownSampleFilter::getLatestRawValue() const {
  return latest_value_.load();
}

// 获取混合值：最新高阶BUSF滤波值 + 下采样趋势
double DownSampleFilter::getLatestHybridValue() {
  double latest_busf = latest_filtered_value_.load();
  double latest_downsampled = 0.0;

  {
    std::lock_guard<std::mutex> lock(low_speed_mtx_);
    if (!low_speed_buffer_.empty()) {
      latest_downsampled = low_speed_buffer_.back();
    } else {
      return latest_busf; // 如果没有下采样数据，返回高阶BUSF滤波值
    }
  }

  // 混合策略：85%最新高阶BUSF滤波值 + 15%下采样值
  return 0.85 * latest_busf + 0.15 * latest_downsampled;
}

// 重置高阶BUSF滤波器
void DownSampleFilter::resetFilter() {
  std::lock_guard<std::mutex> lock(mtx_);
  busf_filter_.reset();
}

// 获取缓冲区状态
size_t DownSampleFilter::getHighSpeedBufferSize() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return high_speed_buffer_.size();
}

size_t DownSampleFilter::getLowSpeedBufferSize() const {
  std::lock_guard<std::mutex> lock(low_speed_mtx_);
  return low_speed_buffer_.size();
}

// 获取采样统计信息
size_t DownSampleFilter::getTotalSampleCount() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return sample_count_;
}

// 内部数据处理函数
int DownSampleFilter::internalPush(double value) {
  // 立即进行高阶BUSF滤波
  double filtered_value = busf_filter_.filter(value);

  // 更新原始值和滤波值
  latest_value_.store(value);
  latest_filtered_value_.store(filtered_value);

  std::lock_guard<std::mutex> lock(mtx_);
  high_speed_buffer_.push_back(filtered_value); // 存储滤波后的值
  sample_count_++;

  // 限制缓冲区大小
  while (high_speed_buffer_.size() > HIGH_SPEED_BUFFER_MAX_SIZE) {
    high_speed_buffer_.pop_front();
  }

  return high_speed_buffer_.size();
}

// 主处理循环：1000Hz数据采集 + 50Hz下采样输出
void DownSampleFilter::processLoop() {
  auto next_high_freq_time = std::chrono::steady_clock::now();
  auto next_low_freq_time = std::chrono::steady_clock::now();

  const auto high_freq_interval =
      std::chrono::microseconds(1000); // 1000Hz = 1ms间隔
  const auto low_freq_interval =
      std::chrono::milliseconds(20); // 50Hz = 20ms间隔

  size_t high_freq_counter = 0; // 1000Hz采样计数器

  while (true) {
    auto now = std::chrono::steady_clock::now();

    // 检查是否需要停止
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!running_) {
        break;
      }
    }

    // 1000Hz 高频数据采集和BUSF滤波
    if (now >= next_high_freq_time) {
      // 调用pusher获取传感器数据
      if (pusher_) {
        try {
          double sensor_value = pusher_();
          internalPush(sensor_value);
          high_freq_counter++;
        } catch (const std::exception &e) {
          // std::cerr << "Sensor read error: " << e.what() << std::endl;
        }
      }

      // 计算下次1000Hz采集时间
      next_high_freq_time += high_freq_interval;

      // 防止时间累积误差
      if (next_high_freq_time < now) {
        next_high_freq_time = now + high_freq_interval;
      }
    }

    // 50Hz 低频下采样处理
    if (now >= next_low_freq_time) {
      std::unique_lock<std::mutex> lock(mtx_);

      // 自适应下采样：根据可用数据量调整
      size_t available_samples = high_speed_buffer_.size();
      if (available_samples > 0) {
        size_t samples_to_process = std::min(available_samples, factor_);

        double sum = 0.0;
        for (size_t i = 0; i < samples_to_process; i++) {
          sum += high_speed_buffer_.front();
          high_speed_buffer_.pop_front();
        }
        double avg = sum / static_cast<double>(samples_to_process);

        // 将下采样结果放入低速缓冲区
        {
          std::lock_guard<std::mutex> low_lock(low_speed_mtx_);
          low_speed_buffer_.push_back(avg);
          while (low_speed_buffer_.size() > MAX_LOW_SPEED_HISTORY) {
            low_speed_buffer_.pop_front();
          }
        }
      }

      lock.unlock();

      next_low_freq_time += low_freq_interval;

      // 防止时间漂移
      if (next_low_freq_time < now) {
        next_low_freq_time = now + low_freq_interval;
      }
    }

    // 精确的微秒级睡眠控制
    auto sleep_until = std::min(next_high_freq_time, next_low_freq_time);
    auto sleep_duration = sleep_until - now;

    if (sleep_duration > std::chrono::microseconds(100)) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    } else if (sleep_duration > std::chrono::microseconds(10)) {
      std::this_thread::sleep_for(sleep_duration);
    }
    // 对于非常短的等待时间，使用忙等待以提高精度
  }
}