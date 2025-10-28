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
    // 初始化级联段历史
    for (auto &section : cascaded_sections_) {
      section.x1 = section.x2 = input;
      section.y1 = section.y2 = input;
    }
    initialized_ = true;
    return input;
  }

  double output = input;

  // 如果使用级联实现
  if (!cascaded_sections_.empty()) {
    for (auto &section : cascaded_sections_) {
      output = section.filter(output);
    }
    return output;
  }

  // 否则使用直接形式
  // 移位历史数组
  for (int i = order_; i > 0; i--) {
    x_history_[i] = x_history_[i - 1];
    y_history_[i] = y_history_[i - 1];
  }
  x_history_[0] = input;

  // 计算输出
  output = 0.0;
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
  // 修正双线性变换 - 与scipy保持一致
  double wc = tan(M_PI * normalized_cutoff / 2.0); // 关键修正：除以2

  // 添加频率范围检查
  if (normalized_cutoff > 0.8) {
    throw std::invalid_argument(
        "Cutoff frequency too close to Nyquist frequency");
  }

  if (order == 1) {
    // 一阶Butterworth滤波器 - scipy兼容
    double norm = 1.0 + wc;
    b_[0] = wc / norm;
    b_[1] = wc / norm;
    a_[0] = 1.0;
    a_[1] = (wc - 1.0) / norm;

  } else if (order == 2) {
    // 二阶Butterworth滤波器 - scipy兼容实现
    double k = wc;
    double k2 = k * k;
    double sqrt2 = M_SQRT2;
    double norm = 1.0 + sqrt2 * k + k2;

    b_[0] = k2 / norm;
    b_[1] = 2.0 * k2 / norm;
    b_[2] = k2 / norm;
    a_[0] = 1.0;
    a_[1] = 2.0 * (k2 - 1.0) / norm;
    a_[2] = (1.0 - sqrt2 * k + k2) / norm;

  } else if (order >= 3) {
    // 高阶使用级联实现
    calculateHighOrderCoefficients(normalized_cutoff, order);
    return;
  }

  // 检查数值有效性
  for (int i = 0; i <= order_; i++) {
    if (!std::isfinite(a_[i]) || !std::isfinite(b_[i])) {
      throw std::runtime_error("Non-finite filter coefficients detected");
    }
  }
}

// 修改高阶滤波器的稳定实现
void ButterworthFilter::calculateHighOrderCoefficients(double normalized_cutoff,
                                                       int order) {
  // 将高阶滤波器分解为多个二阶段
  int num_second_order = order / 2;
  int has_first_order = order % 2;

  // 重新初始化为级联形式
  cascaded_sections_.clear();

  // 为奇数阶添加一阶段
  if (has_first_order) {
    CascadeSection first_order;
    double k = tan(M_PI * normalized_cutoff);
    double norm = 1.0 + k;

    first_order.b0 = k / norm;
    first_order.b1 = k / norm;
    first_order.b2 = 0.0;
    first_order.a1 = (k - 1.0) / norm;
    first_order.a2 = 0.0;
    first_order.x1 = first_order.x2 = 0.0;
    first_order.y1 = first_order.y2 = 0.0;

    cascaded_sections_.push_back(first_order);
  }

  // 添加二阶段 - 使用标准Butterworth极点
  for (int i = 0; i < num_second_order; i++) {
    CascadeSection second_order;

    // 计算Butterworth极点角度
    double theta = M_PI * (2.0 * i + 1.0 + has_first_order) / (2.0 * order);

    // 计算二阶段系数
    double k = tan(M_PI * normalized_cutoff);
    double k2 = k * k;

    // Butterworth极点实部和虚部
    double sigma = cos(theta); // 实部 (负值，左半平面)
    double omega = sin(theta); // 虚部

    // 对于二阶段，使用Q因子方法
    double Q = 1.0 / (2.0 * sigma); // Q因子
    double norm = 1.0 + k / Q + k2;

    second_order.b0 = k2 / norm;
    second_order.b1 = 2.0 * k2 / norm;
    second_order.b2 = k2 / norm;
    second_order.a1 = 2.0 * (k2 - 1.0) / norm;
    second_order.a2 = (1.0 - k / Q + k2) / norm;
    second_order.x1 = second_order.x2 = 0.0;
    second_order.y1 = second_order.y2 = 0.0;

    // 验证二阶段稳定性
    if (std::abs(second_order.a1) >= 2.0 || std::abs(second_order.a2) >= 1.0) {
      throw std::runtime_error("Unstable cascaded section detected");
    }

    cascaded_sections_.push_back(second_order);
  }

  // 对于级联实现，主系数数组设为单位值
  std::fill(b_.begin(), b_.end(), 0.0);
  std::fill(a_.begin(), a_.end(), 0.0);
  b_[0] = 1.0;
  a_[0] = 1.0;
}

// 修改稳定性检查，对级联实现跳过主系数检查
void ButterworthFilter::checkStability() {
  // 如果使用级联实现，检查各个段的稳定性
  if (!cascaded_sections_.empty()) {
    for (const auto &section : cascaded_sections_) {
      // 只检查数值有效性，不检查阈值
      if (!std::isfinite(section.b0) || !std::isfinite(section.b1) ||
          !std::isfinite(section.b2) || !std::isfinite(section.a1) ||
          !std::isfinite(section.a2)) {
        throw std::runtime_error("Non-finite cascaded section coefficients");
      }
    }
    return;
  }

  // 对于直接形式，只检查数值有效性
  for (int i = 0; i <= order_; i++) {
    if (!std::isfinite(a_[i]) || !std::isfinite(b_[i])) {
      throw std::runtime_error("Non-finite filter coefficients detected");
    }
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