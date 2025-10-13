#include "ft_sensor/force_control.h"
template <ArithmeticWithDouble T>
int AdmittanceController<T>::setPositionSensor(
    milliseconds dt, std::function<T()> position_sensor) {
  p_dt_ = dt;
  p_sensor_ = position_sensor;
  thread_stop_ = 0;
  psensor_thread_ = std::thread([this]() {
    while (thread_stop_ == 0) {
      auto now = steady_clock::now();
      T position = p_sensor_();
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        position_data_.push_back({now, position});
        while (position_data_.size() > data_max_) {
          position_data_.pop_front();
        }
      }
      std::this_thread::sleep_until(now + p_dt_);
    }
  });
  return 1;
}
template <ArithmeticWithDouble T>
int AdmittanceController<T>::setForceSensor(milliseconds dt,
                                            std::function<T()> force_sensor) {
  f_dt_ = dt;
  f_sensor_ = force_sensor;
  thread_stop_ = 0;
  fsensor_thread_ = std::thread([this]() {
    while (thread_stop_ == 0) {
      auto now = steady_clock::now();
      T force = f_sensor_();
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        force_data_.push_back({now, force});
        while (force_data_.size() > data_max_) {
          force_data_.pop_front();
        }
      }
      std::this_thread::sleep_until(now + f_dt_);
    }
  });
  return 1;
}
template <ArithmeticWithDouble T>
int AdmittanceController<T>::setPositionUpdate(
    milliseconds dt, std::function<int(T)> position_updater) {
  p_dt_ = dt;
  thread_stop_ = 0;
  position_update_thread_ = std::thread([this, position_updater]() {
    while (thread_stop_ == 0) {
      if (position_data_.size() < 3 || force_data_.empty()) {
        continue;
      }
      auto now = steady_clock::now();
      DataStamped p, p_1, p_2, f;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        p = position_data_.back();
        p_1 = position_data_.at(position_data_.size() - 2);
        p_2 = position_data_.at(position_data_.size() - 3);
        f = force_data_.back();
      }
      if (duration_cast<milliseconds>(p.time - p_1.time).count() == 1000) {
        // 当数据太老时，不进行计算
        continue;
      }
      double dt1 = duration_cast<nanoseconds>(p.time - p_1.time).count() / 1e9;
      double dt2 =
          duration_cast<nanoseconds>(p_1.time - p_2.time).count() / 1e9;
      auto p_dot = (p.value - p_1.value) / dt1;
      auto p_dot_dot = (p_dot - (p_1.value - p_2.value) / dt2) / dt1;
      T p_target = (f.value - kv_ * p_dot - m_ * k_ * p_dot_dot) / k_;
      int ret = position_updater(p_target);
      std::this_thread::sleep_until(now + p_dt_);
    }
  });
  return 1;
}