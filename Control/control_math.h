#ifndef CONTROL_MATH_H
#define CONTROL_MATH_H

#include <limits.h>
#include <stdint.h>

/* Return the magnitude of a signed 64-bit value without signed overflow. */
static inline uint64_t control_abs_i64(int64_t value)
{
    if (value < 0) {
        return (uint64_t)(-(value + 1)) + 1u;
    }
    return (uint64_t)value;
}

static inline uint64_t control_abs_diff_i64(int64_t left, int64_t right)
{
    if (left >= right) {
        return (uint64_t)left - (uint64_t)right;
    }
    return (uint64_t)right - (uint64_t)left;
}

static inline uint64_t control_abs_diff_u64(uint64_t left, uint64_t right)
{
    return left >= right ? left - right : right - left;
}

static inline uint64_t control_add_u64_saturating(uint64_t left, uint64_t right)
{
    if (UINT64_MAX - left < right) {
        return UINT64_MAX;
    }
    return left + right;
}

static inline int control_abs_int_saturating(int value)
{
    uint64_t magnitude = control_abs_i64((int64_t)value);

    return magnitude > (uint64_t)INT_MAX ? INT_MAX : (int)magnitude;
}

static inline float control_wrap_degrees(float angle)
{
    while (angle >= 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static inline float control_heading_error_degrees(float reality, float target)
{
    return control_wrap_degrees(reality - target);
}

static inline float control_heading_abs_error_degrees(float reality, float target)
{
    float error = control_heading_error_degrees(reality, target);

    return error < 0.0f ? -error : error;
}

static inline uint8_t control_distance_reached(
    uint64_t traveled, uint64_t target, uint64_t tolerance)
{
    if (traveled >= target) {
        return 1u;
    }
    return (target - traveled) <= tolerance;
}

#endif
