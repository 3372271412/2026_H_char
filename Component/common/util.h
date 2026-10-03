/**
 * @file    util.h
 * @brief   跨模块共享的小型内联工具函数。
 *
 * @details header-only：仅含 static inline 函数，无需编译为 .c，
 *          通过 IncludePath 中的 `../Component` 即可 `#include "common/util.h"`。
 *          用于消除各模块中重复的 abs/clamp 等小工具。
 */
#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H

#include <limits.h>
#include <stdint.h>

/** @brief 32 位有符号整数取绝对值（避免各模块重复实现）。 */
static inline int32_t util_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/** @brief 将 64 位整数安全收敛到 int32_t 范围（溢出截断到极值）。 */
static inline int32_t util_clamp_i64_to_i32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

#endif /* COMMON_UTIL_H */