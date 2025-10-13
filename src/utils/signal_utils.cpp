#include "utils/signals_utils.h"
#include <deque>
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
