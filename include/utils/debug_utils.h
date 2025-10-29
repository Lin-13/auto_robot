#pragma once
#include <algorithm> // 用于std::min
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#ifndef ENABLE_PROFILER
#define ENABLE_PROFILER 1
#endif

struct ThreadFuncStats {
  size_t call_count = 0;
  double total_time = 0.0;
  double first_call_time = 0.0;
  double last_call_time = 0.0;
};

// 启动时间
static double g_program_start_time = []() {
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double, std::milli>(now.time_since_epoch())
      .count();
}();

class FunctionProfiler {
private:
  // 函数名 → 线程ID → 该线程的函数统计数据
  std::unordered_map<std::string,
                     std::unordered_map<std::thread::id, ThreadFuncStats>>
      stats;
  std::mutex mtx;

  static double get_current_time() {
    auto now = std::chrono::high_resolution_clock::now();
    double now_us =
        std::chrono::duration<double, std::milli>(now.time_since_epoch())
            .count();
    return now_us - g_program_start_time;
  }

public:
  static FunctionProfiler &get_instance() {
    static FunctionProfiler instance;
    return instance;
  }

  void enter(const std::string &func_name, double &start_time) {
    start_time = get_current_time();
  }

  void exit(const std::string &func_name, double start_time) {
    double elapsed = get_current_time() - start_time;
    std::thread::id tid = std::this_thread::get_id();
    double current_time = get_current_time(); // 本次调用结束时间

    std::lock_guard<std::mutex> lock(mtx);
    auto &thread_stats = stats[func_name][tid];
    thread_stats.call_count++;
    thread_stats.total_time += elapsed;

    if (thread_stats.call_count == 1) {
      thread_stats.first_call_time =
          current_time - elapsed; // 首次调用的开始时间
    }
    thread_stats.last_call_time = current_time - elapsed; // 末次调用的结束时间
  }

  double calculate_frequency(const ThreadFuncStats &stats_data) {
    if (stats_data.call_count < 2) {
      return 0.0;
    }
    double total_duration_s =
        (stats_data.last_call_time - stats_data.first_call_time) / 1e3;
    if (total_duration_s < 1e-6) {
      return 0.0;
    }
    return stats_data.call_count / total_duration_s;
  }

  void print_stats() {
    std::lock_guard<std::mutex> lock(mtx);

    // 标题与分隔线
    std::string line(51, '=');
    std::cout << "\n" << line << "   Profile Result   " << line << std::endl;

    // 列宽定义（根据最长内容动态调整或固定）
    const int col_func = 50;  // 函数名
    const int col_tid = 12;   // 线程ID
    const int col_count = 15; // 调用次数
    const int col_total = 15; // 总耗时(ms)
    const int col_avg = 15;   // 平均耗时(ms)
    const int col_freq = 15;  // 调用频率(次/秒)

    // 表头（使用右对齐增强数字列的可读性）
    std::cout << std::left << std::setw(col_func) << "Name"
              << std::setw(col_tid) << "ThreadID" << std::right
              << std::setw(col_count) << "Count   " << std::setw(col_total)
              << "   Total (ms)   " << std::setw(col_avg) << "   Avg (ms)"
              << std::setw(col_freq) << "Freq (Hz)" << std::endl;

    // 表头分隔线
    std::cout << std::left << std::setw(col_func) << std::string(col_func, '-')
              << std::setw(col_tid) << std::string(col_tid, '-') << std::right
              << std::setw(col_count) << std::string(col_count, '-')
              << std::setw(col_total) << std::string(col_total, '-')
              << std::setw(col_avg) << std::string(col_avg, '-')
              << std::setw(col_freq) << std::string(col_freq, '-') << std::endl;

    // 遍历输出统计数据
    for (const auto &[func_name, thread_map] : stats) {
      for (const auto &[tid, stats_data] : thread_map) {
        const double avg_time =
            stats_data.call_count > 0
                ? stats_data.total_time / stats_data.call_count
                : 0.0;
        const double frequency = calculate_frequency(stats_data);

        // 函数名左对齐，数字右对齐，控制精度
        std::cout << std::left << std::setw(col_func)
                  << func_name.substr(0, col_func - 1) // 防止超长截断
                  << std::setw(col_tid) << tid << std::right
                  << std::setw(col_count) << stats_data.call_count
                  << std::setw(col_total) << std::fixed << std::setprecision(2)
                  << stats_data.total_time << std::setw(col_avg) << std::fixed
                  << std::setprecision(2) << avg_time << std::setw(col_freq)
                  << std::fixed << std::setprecision(2) << frequency
                  << std::endl;
      }
    }
    std::string end_line(122, '=');
    std::cout << end_line << std::endl;
  }
};

class ProfilerGuard {
private:
  std::string func_name;
  double start_time;
  bool stop;

public:
  ProfilerGuard(const std::string &name, double start)
      : func_name(name), start_time(start), stop(false) {}
  void Stop() {
    if (stop == false) {
      FunctionProfiler::get_instance().exit(func_name, start_time);
      stop = true;
    }
  }
  ~ProfilerGuard() { Stop(); }
};

// 埋点
#if ENABLE_PROFILER
#define PROFILE()                                                              \
  double __profiler_start_time;                                                \
  FunctionProfiler::get_instance().enter(__FUNCTION__, __profiler_start_time); \
  ProfilerGuard __profiler_guard(__FUNCTION__, __profiler_start_time);
#define PROFILE_NAME(name)                                                     \
  double __profiler_start_time;                                                \
  FunctionProfiler::get_instance().enter(name, __profiler_start_time);         \
  ProfilerGuard __profiler_guard(name, __profiler_start_time);
#define PROFILE_NAME_STOP(name) __profiler_guard.Stop();
#define PRINT_PROFILER() FunctionProfiler::get_instance().print_stats();
#else
#define PROFILE()
#define PROFILE_NAME(name)
#define PROFILE_NAME_STOP(name)
#define PRINT_PROFILER()
#endif