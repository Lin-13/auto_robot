#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
using namespace std::chrono;

// 算术类型约束：支持与double的基本运算
template <typename T>
concept ArithmeticWithDouble = requires(T a, T b, double d) {
  { a + b } -> std::convertible_to<T>;
  { a - b } -> std::convertible_to<T>;
  { a *d } -> std::convertible_to<T>;
  { a / d } -> std::convertible_to<T>;
  { d *a } -> std::convertible_to<T>;
};
// template <typename Coef, typename T>
// concept CoefCompatibleWith = requires(Coef c, T t) {
//   { c *t } -> std::convertible_to<T>;
//   []<bool IsEigen = std::is_base_of_v<Eigen::EigenBase<T>, T>>() {
//     if constexpr (IsEigen) {
//       return requires { t *c.inverse(); };
//     } else {
//       return requires { t / c; };
//     }
//   };
// };

/**
 * @brief 一维导纳控制器
 * 实现弹簧阻尼系统动力学方程：m*x_dot_dot+ kv*x_dot+ k*x = f
 */
template <ArithmeticWithDouble T, typename CoefType>
// requires CoefCompatibleWith<CoefType, T>
class AdmittanceController {
public:
  struct DataStamped {
    steady_clock::time_point time;
    T value;
  };

  // 构造函数：初始化物理参数和期望力
  AdmittanceController(CoefType m, CoefType kv, CoefType k,
                       T desired_force = T{}, T desired_pos = T{})
      : m_(m), kv_(kv), k_(k), desired_force_(desired_force),
        desired_pos_(desired_pos) {
    if constexpr (std::is_scalar_v<CoefType>) {
      if (m <= 0 || kv < 0 || k < 0) {
        throw std::invalid_argument("标量物理参数：m必须>0，kv和k必须≥0");
      }
    } else if constexpr (std::is_base_of_v<Eigen::EigenBase<CoefType>,
                                           CoefType>) {
      if (m.rows() != m.cols()) {
        throw std::invalid_argument("M error");
      }
      if (kv.rows() != kv.cols()) {
        throw std::invalid_argument("Kv error");
      }
      if (k.rows() != k.cols()) {
        throw std::invalid_argument("K error");
      }
    }
    if constexpr (std::is_base_of_v<Eigen::EigenBase<T>, T> &&
                  std::is_base_of_v<Eigen::EigenBase<CoefType>, CoefType>) {
      // static_assert(T::RowsAtCompileTime == CoefType::ColsAtCompileTime,
      //               "T的行数（维度）必须与CoefType的列数匹配");
      // static_assert(CoefType::RowsAtCompileTime ==
      // CoefType::ColsAtCompileTime,
      //               "CoefType必须是方阵");
    }
  }

  // 禁止拷贝构造和赋值（线程安全考虑）
  AdmittanceController(const AdmittanceController &) = delete;
  AdmittanceController &operator=(const AdmittanceController &) = delete;

  /**
   * @brief 设置位置传感器
   * @param position_sensor 位置传感器函数
   * @return 1成功，-1失败（传感器为空）
   */
  int setPositionSensor(std::function<T()> position_sensor) {
    if (!position_sensor)
      return -1;
    p_sensor_ = std::move(position_sensor);
    return 1;
  }

  /**
   * @brief 设置力传感器
   * @param force_sensor 力传感器函数
   * @return 1成功，-1失败（传感器为空）
   */
  int setForceSensor(std::function<T()> force_sensor) {
    if (!force_sensor)
      return -1;
    f_sensor_ = std::move(force_sensor);
    return 1;
  }

  /**
   * @brief 设置位置更新器
   * @param position_updater 位置更新器函数
   * @return 1成功，-1失败（更新器为空）
   */
  int setPositionUpdate(std::function<int(T)> position_updater) {
    if (!position_updater)
      return -1;
    position_updater_ = std::move(position_updater);
    return 1;
  }

  /**
   * @brief 启动周期更新线程
   * @param dt 更新周期（毫秒）
   * @return 1成功，-1已在运行，-2参数无效
   */
  int start(milliseconds dt) {
    if (thread_stop_ == 0)
      return -1; // 已在运行
    if (dt <= 0ms)
      return -2;

    thread_stop_ = 0;
    position_update_thread_ =
        std::thread(&AdmittanceController::updateLoop, this, dt);
    return 1;
  }

  /**
   * @brief 停止更新线程
   */
  void stop() {
    if (thread_stop_ == 0) {
      thread_stop_ = 1;
      if (position_update_thread_.joinable()) {
        position_update_thread_.join();
      }
    }
  }

  /**
   * @brief 单次更新计算
   * @return 0成功，-1传感器/更新器未设置，-2数据不足，-3更新失败
   */
  int updateOnce() {
    // 检查传感器和更新器是否就绪
    if (!p_sensor_ || !f_sensor_ || !position_updater_) {
      return -1;
    }

    // 读取传感器数据并缓存
    try {
      const auto now = steady_clock::now();
      const T p_val = p_sensor_();
      const T f_val = f_sensor_();

      std::lock_guard<std::mutex> lock(data_mutex_);
      position_data_.push_back({now, p_val});
      force_data_.push_back({now, f_val});

      // 限制缓存大小
      while (position_data_.size() > data_max_)
        position_data_.pop_front();
      while (force_data_.size() > data_max_)
        force_data_.pop_front();
    } catch (...) {
      return -1; // 传感器读取失败
    }

    // 检查数据是否足够
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (position_data_.size() < 3 || force_data_.empty()) {
      return -2;
    }

    // 提取计算所需数据
    const DataStamped &p = position_data_.back();
    const DataStamped &p_1 = position_data_[position_data_.size() - 2];
    const DataStamped &p_2 = position_data_[position_data_.size() - 3];
    const DataStamped &f = force_data_.back();

    // 检查数据时效性
    const auto dt_p = duration_cast<milliseconds>(p.time - p_1.time).count();
    const auto dt_p1 = duration_cast<milliseconds>(p_1.time - p_2.time).count();
    if (dt_p <= 0 || dt_p1 <= 0 || dt_p > 1000 || dt_p1 > 1000) {
      return -2; // 无效时间间隔
    }

    // 计算速度和加速度（转换为m/s和m/s²）
    const T v = (p.value - p_1.value) / static_cast<double>(dt_p) * 1000.0;
    const T v_prev =
        (p_1.value - p_2.value) / static_cast<double>(dt_p1) * 1000.0;
    const T a = (v - v_prev) / static_cast<double>(dt_p) * 1000.0;

    // 导纳方程计算：M*a_d + B*v + K*x = F_ext
    const T F_ext = f.value - desired_force_;
    T a_d;
    if constexpr (std::is_base_of_v<Eigen::EigenBase<T>, T>) {
      a_d = m_.inverse() * (F_ext - kv_ * v - k_ * (p.value - desired_pos_));
    } else {
      a_d = (F_ext - kv_ * v - k_ * (p.value - desired_pos_)) / m_;
    }

    // 积分计算期望位置（使用当前时间间隔）
    const double dt_sec = static_cast<double>(dt_p) / 1000.0; // 转换为秒
    const T v_d = v + a_d * dt_sec;
    const T x_d = p.value + v_d * dt_sec;

    // 执行位置更新
    current_pos_ = x_d;
    if (position_updater_(x_d) != 0) {
      return -3;
    }

    return 0;
  }
  // get set
  T getPos() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return p_sensor_.back().value;
  }
  T getDesiredForce() {
    std::lock_guard<std::mutex> lock(param_mutex_);
    return desired_force_;
  }
  T getDesiredPos() {
    std::lock_guard<std::mutex> lock(param_mutex_);
    return desired_pos_;
  }
  void setDesiredForce(T desired_force) {
    std::lock_guard<std::mutex> lock(param_mutex_);
    desired_force_ = desired_force;
  }
  void setDesiredPos(T desired_pos) {
    std::lock_guard<std::mutex> lock(param_mutex_);
    desired_pos_ = desired_pos;
  }
  /**
   * @brief
   * 使用update函数的更新位置作为位置传感器，避免使用实际位置导致位置突变和震荡
   *
   * @param initial_pos
   */
  void setVirtualPositionSensor(T initial_pos) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_pos_ = initial_pos;
    p_sensor_ = [this]() { return this->current_pos_; };
    return;
  }
  /**
   * @brief 设置最大缓存数据量
   * @param max_size 最大缓存大小
   */
  void setMaxDataSize(size_t max_size) {
    if (max_size < 3)
      max_size = 3; // 确保至少能存储3个位置数据
    data_max_ = max_size;
  }

  ~AdmittanceController() {
    stop(); // 确保线程正确停止
  }

private:
  // 线程循环：周期性调用更新
  void updateLoop(milliseconds dt) {
    while (thread_stop_ == 0) {
      const auto start = steady_clock::now();
      updateOnce(); // 执行单次更新
      // 计算剩余睡眠时间（确保周期稳定）
      const auto elapsed =
          duration_cast<milliseconds>(steady_clock::now() - start);
      const auto sleep_time = (elapsed < dt) ? (dt - elapsed) : 0ms;
      std::this_thread::sleep_for(sleep_time);
    }
  }

  // 物理参数
  CoefType m_;      // 质量
  CoefType kv_;     // 阻尼系数
  CoefType k_;      // 刚度系数
  T desired_force_; // 期望力
  T desired_pos_;   // 期望位置
  T current_pos_;   // 当前期望位置

  // 传感器和执行器
  std::function<T()> p_sensor_;            // 位置传感器
  std::function<T()> f_sensor_;            // 力传感器
  std::function<int(T)> position_updater_; // 位置更新器

  // 数据缓存
  std::deque<DataStamped> position_data_; // 位置历史数据
  std::deque<DataStamped> force_data_;    // 力历史数据
  std::atomic<size_t> data_max_ = 100;    // 最大缓存大小

  // 线程控制
  std::atomic<int> thread_stop_ = 1; // 线程停止标志（1:停止, 0:运行）
  std::thread position_update_thread_; // 更新线程

  // 同步机制
  mutable std::mutex data_mutex_;  // 数据缓存互斥锁
  mutable std::mutex param_mutex_; // 参数修改互斥锁
};

// 常用类型别名
using AdmittanceController1d = AdmittanceController<double, double>;
using AdmittanceController3d =
    AdmittanceController<Eigen::Vector3d, Eigen::Matrix3d>;