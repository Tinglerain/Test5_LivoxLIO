// 时间工具（新人实现）。
#pragma once

#include <vector>

namespace small_glim {

/// 用历史帧间隔估计扫描时长：返回中位数。
///
/// 约定：
///   - 输入为空时返回 -1；
///   - 不修改输入；
///   - 偶数个元素时取中间偏后的那个；
///   - 只做统计，不判断范围是否合法。
double medianScanDuration(const std::vector<double>& durations);

} // namespace small_glim
