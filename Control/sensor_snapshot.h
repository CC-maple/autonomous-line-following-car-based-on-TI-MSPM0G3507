#ifndef SENSOR_SNAPSHOT_H
#define SENSOR_SNAPSHOT_H

#include <stddef.h>
#include <stdint.h>

static inline int8_t sensor_snapshot_decode(
    uint32_t raw_pins, const uint32_t *pin_masks,
    int8_t *values, size_t channel_count)
{
    int8_t sum = 0;
    size_t channel;

    for (channel = 0u; channel < channel_count; ++channel) {
        values[channel] = (raw_pins & pin_masks[channel]) != 0u ? 1 : 0;
        sum += values[channel];
    }
    return sum;
}

#endif
