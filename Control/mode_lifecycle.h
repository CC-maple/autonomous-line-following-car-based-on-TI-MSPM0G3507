#ifndef MODE_LIFECYCLE_H
#define MODE_LIFECYCLE_H

#include <stdint.h>

#define MODE_LIFECYCLE_IDLE 0u
#define MODE_LIFECYCLE_ENTER 1u
#define MODE_LIFECYCLE_EXIT 2u
#define MODE_LIFECYCLE_ABORT 3u
#define MODE_LIFECYCLE_INVALID 4u
#define MODE_LIFECYCLE_CONTINUE 5u

typedef struct {
    uint8_t active_mode;
} ModeLifecycle;

static inline void mode_lifecycle_init(ModeLifecycle *lifecycle)
{
    lifecycle->active_mode = 0u;
}

static inline uint8_t mode_lifecycle_is_valid_mode(uint8_t mode)
{
    return mode >= 1u && mode <= 7u;
}

static inline uint8_t mode_lifecycle_step(
    ModeLifecycle *lifecycle, uint8_t selected_mode, uint8_t begin)
{
    if (begin == 0u) {
        if (lifecycle->active_mode != 0u) {
            lifecycle->active_mode = 0u;
            return MODE_LIFECYCLE_EXIT;
        }
        return MODE_LIFECYCLE_IDLE;
    }

    if (selected_mode == 0u) {
        if (lifecycle->active_mode != 0u) {
            lifecycle->active_mode = 0u;
            return MODE_LIFECYCLE_EXIT;
        }
        return MODE_LIFECYCLE_INVALID;
    }

    if (!mode_lifecycle_is_valid_mode(selected_mode)) {
        lifecycle->active_mode = 0u;
        return MODE_LIFECYCLE_INVALID;
    }

    if (lifecycle->active_mode == 0u) {
        lifecycle->active_mode = selected_mode;
        return MODE_LIFECYCLE_ENTER;
    }

    if (lifecycle->active_mode != selected_mode) {
        lifecycle->active_mode = 0u;
        return MODE_LIFECYCLE_ABORT;
    }

    return MODE_LIFECYCLE_CONTINUE;
}

#endif
