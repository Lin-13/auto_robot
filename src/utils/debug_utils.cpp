#include "utils/debug_utils.h"
double tictok_timer(int command) {
  static auto last = std::chrono::high_resolution_clock::now();
  if (command == 0) {
    last = std::chrono::high_resolution_clock::now();
    return 0.0;
  } else {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = now - last;
    last = now;
    return duration.count();
  }
}
double tic() { return tictok_timer(0); }
double toc() { return tictok_timer(1); }