#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
/**
 * @brief 定时器 多线程定时触发器，用于RobotController
 *
 */
class Timer {
public:
  enum class State {
    Running,
    Stopped,
    Paused,
  };

  /**
   * @brief Timer callback
   * @return int
   * 0:success
   * -1:fail,stop timer
   * other: warning,print
   */
  using timer_callback = std::function<int()>;
  /**
   * @brief Timer monitor 当callback不为 0 和 -1时调用该函数进行处理
   * @param name timer name
   * @param code error code
   */
  using timer_monitor = std::function<void(int, std::string)>;
  using Ptr = std::shared_ptr<Timer>;

  ~Timer();
  State state();
  void start();
  void pause();
  void resume();
  void stop();
  static Ptr create(std::string name, timer_callback cb,
                    std::chrono::milliseconds interval,
                    timer_monitor monitor = nullptr) {
    return Ptr(new Timer(name, cb, interval, monitor));
  }

protected:
  void run_thread(std::chrono::milliseconds interval = 0s);

private:
  Timer(std::string name, timer_callback cb, std::chrono::milliseconds interval,
        timer_monitor monitor = nullptr)
      : name_(name), cb_(cb), interval_(interval), monitor_(monitor),
        state_(State::Stopped) {}
  std::string name_;
  timer_callback cb_;
  std::chrono::milliseconds interval_;
  timer_monitor monitor_;
  std::thread thread_;
  std::condition_variable cv_;
  std::mutex mtx_;
  State state_ = State::Stopped;
};
