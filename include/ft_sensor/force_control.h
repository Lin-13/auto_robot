#include <Eigen/Core>
#include <chrono>
#include <deque>
#include <functional>
#include <thread>
using namespace std::chrono_literals;
using namespace std::chrono;
template <typename T>
concept ArithmeticWithDouble = requires(T a, T b, double d) {
  { a + b } -> std::convertible_to<T>; // 支持 T + T
  { a - b } -> std::convertible_to<T>; // 支持 T - T
  { a *d } -> std::convertible_to<T>;  // 支持 T * double
  { a / d } -> std::convertible_to<T>; // 支持 T / double
  { d *a } -> std::convertible_to<T>;  // 支持 double * T
};
/**
 * @brief 一维维导纳控制器
 *
 * 实现弹簧阻尼系统动力学方程：
 * \f$ m \cdot \ddot{p} + k_v \cdot \dot{p} + k \cdot p = f \f$
 *
 * - 输入:
 *
 *   -- PositionSensor: 位置传感器 [function () -> double]
 *
 *   -- ForceSensor: 力传感器 [function () -> double]
 *
 * - 输出:
 *
 *   -- PositionUpdater: 位置更新器 [function (double) -> int]
 *
 * - 类型约束:
 *   -- T 必须支持 +、- 运算符
 *
 *   -- T 必须支持与 double 类型的 *、/ 运算符
 *
 * @param m    质量
 * @param kv   阻尼系数
 * @param k    弹簧系数
 */
template <ArithmeticWithDouble T> class AdmittanceController {
public:
  struct DataStamped {
    steady_clock::time_point time;
    T value;
  };
  AdmittanceController(double m, double kv, double k) : m_(m), kv_(kv), k_(k) {}
  /**
   * @brief 设置位置传感器
   *
   * @param dt 采样时间
   * @param position_sensor 位置传感器函数
   * @return int 1 成功 -1 失败
   */
  int setPositionSensor(milliseconds dt, std::function<T()> position_sensor);
  /**
   * @brief 设置力传感器
   *
   * @param dt 采样时间
   * @param force_sensor 力传感器函数
   * @return int 1 成功 -1 失败
   */
  int setForceSensor(milliseconds dt, std::function<T()> force_sensor);
  /**
   * @brief 设置位置更新器
   *
   * @param dt 采样时间
   * @param position_updater 位置更新器函数
   * @return int 1
   */
  int setPositionUpdate(milliseconds dt,
                        std::function<int(T)> position_updater);
  /**
   * @brief 启动控制器
   *
   * @return int 1
   */

  int start() {
    thread_stop_ = 0;
    position_update_start_ = 1;
    return 1;
  }
  /**
   * @brief 停止控制器
   *
   * @return int 1
   */
  int stop() {
    position_update_start_ = 0;
    thread_stop_ = 1;
    psensor_thread_.join();
    fsensor_thread_.join();
    position_update_thread_.join();
    return 1;
  }
  ~AdmittanceController() { stop(); }

private:
  std::chrono::milliseconds p_dt_;
  std::chrono::milliseconds f_dt_;
  std::function<T()> p_sensor_;
  std::function<T()> f_sensor_;
  std::mutex data_mutex_;
  std::thread psensor_thread_;
  std::thread fsensor_thread_;
  std::thread position_update_thread_;
  std::deque<DataStamped> position_data_;
  std::deque<DataStamped> force_data_;
  std::atomic<std::size_t> data_max_ = 100;

  std::atomic<int> position_update_start_ = 0;
  std::atomic<int> thread_stop_ = 0;
  double m_;
  double kv_;
  double k_;
};
using AdmittanceController1d = AdmittanceController<double>;
using AdmittanceController3d = AdmittanceController<Eigen::Vector3d>;