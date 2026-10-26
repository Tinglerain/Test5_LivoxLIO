// 帧时长估计（已给定，不是 TODO）。
#include <small_glim/preprocess/time_utils.hpp>

#include <algorithm>

namespace small_glim {

double medianScanDuration(const std::vector<double>& durations) {
    if (durations.empty()) return -1.0;

    std::vector<double> sorted = durations;
    std::nth_element(
        sorted.begin(),
        sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2),
        sorted.end());
    return sorted[sorted.size() / 2];
}

} // namespace small_glim
