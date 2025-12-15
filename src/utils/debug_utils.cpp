#include "utils/debug_utils.h"
double tictok_timer(int command) {
  static auto last = std::chrono::high_resolution_clock::now();
  if (command == 0) {
    last = std::chrono::high_resolution_clock::now();
    return 0.0;
  } else {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = now - last;
    // 不再重置 last，这样多次调用 toc() 返回的是累计时间
    return duration.count();
  }
}
double tic() { return tictok_timer(0); }
double toc() { return tictok_timer(1); }