#include "robot_interface/timer.h"
Timer::~Timer() { stop(); }
Timer::State Timer::state() { return state_; }
void Timer::start() {
  if (state_ == State::Running) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    state_ = State::Running;
  }
  thread_ = std::thread(&Timer::run_thread, this, interval_);
}
void Timer::pause() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (state_ == State::Running) {
      state_ = State::Paused;
    }
  }
  cv_.notify_one();
}
void Timer::resume() {
  {
    std::unique_lock<std::mutex> lock(mtx_);
    if (state_ == State::Paused) {
      state_ = State::Running;
    }
  }
  cv_.notify_one();
}
void Timer::stop() {
  {
    std::unique_lock<std::mutex> lock(mtx_);
    state_ = State::Stopped;
  }
  std::this_thread::sleep_for(1ms);
  if (thread_.joinable()) {
    thread_.join();
  }
}
void Timer::run_thread(std::chrono::milliseconds interval) {
  while (true) {
    auto start_time = std::chrono::high_resolution_clock::now();
    {
      std::unique_lock<std::mutex> lock(mtx_);
      State state = state_;
      if (state == State::Stopped) {
        break;
      }
      if (state == State::Paused) {
        cv_.wait(lock, [this]() { return state_ != State::Paused; });
        // wait 等待被唤醒
      }
    }
    int ret = cb_();
    if (ret == -1) {
      break;
    }
    if (ret != 0 && ret != -1) {
      if (monitor_) {
        monitor_(ret, name_);
      }
    }
    // auto end_time = std::chrono::steady_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     end_time - start_time);
    // std::this_thread::sleep_for(interval - duration);
    std::this_thread::sleep_until(start_time + interval - 1ms);
    auto end = start_time + interval;
    while (std::chrono::high_resolution_clock::now() < end) {
      // 空循环，等待剩余时间
    }
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    state_ = State::Stopped;
  }
}