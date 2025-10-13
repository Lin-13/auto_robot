#include <Eigen/Core>
#include <deque>
using std::deque;
using std::vector;
namespace SignalFilterType {
const vector<double> smooth_filter_3{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
const vector<double> smooth_filter_phase_3{0.5, 0.25, 0.25};
const vector<double> smooth_filter_5{0.1, 0.2, 0.4, 0.2, 0.1};
const vector<double> smooth_filter_phase_5{3.0 / 8.0, 2.0 / 8.0, 1.0 / 8.0,
                                           1.0 / 8.0, 1.0 / 8.0};
const vector<double> diff1_filter_3{1.0 / 2.0, 0.0, -1.0 / 2.0};
const vector<double> diff1_filter_phase_3{2.0 / 3, -1.0 / 3, -1.0 / 3};
const vector<double> diff1_filter_5{2.0 / 10.0, 1.0 / 10.0, 0, -1.0 / 10.0,
                                    -2.0 / 10.0};
const vector<double> diff1_filter_phase_5{3.0 / 12.0, 2.0 / 12.0, 0,
                                          -1.0 / 12.0, -2.0 / 12.0};
const vector<double> diff2_filter_3{1.0, -2.0, 1.0};
const vector<double> diff2_filter_phase_3{1.0, -2.0, 1.0};
const vector<double> diff2_filter_5{1.0, -4.0, 6.0, -4.0, 1.0};
const vector<double> diff2_filter_phase_5{2.0 / 2.0, -5.0 / 2.0, 4.0 / 2.0, 0,
                                          -1.0 / 2.0};
}; // namespace SignalFilterType
deque<double> signalFilter(const deque<double> &input,
                           const vector<double> &filter);
