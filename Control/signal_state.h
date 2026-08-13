#ifndef SIGNAL_STATE_H
#define SIGNAL_STATE_H

#include <stdint.h>

#define SIGNAL_TICK_NONE 0u
#define SIGNAL_TICK_FINISHED 1u
#define SIGNAL_TICK_STARTED 2u

typedef struct {
    uint16_t ticks_remaining;
    uint16_t duration_ticks;
    uint8_t pending_requests;
    uint8_t gap_pending;
} SignalState;

static inline void signal_state_init(SignalState *state)
{
    state->ticks_remaining = 0u;
    state->duration_ticks = 0u;
    state->pending_requests = 0u;
    state->gap_pending = 0u;
}

static inline uint8_t signal_state_request(SignalState *state, uint16_t duration_ticks)
{
    if (duration_ticks == 0u) {
        return 0u;
    }
    if (state->ticks_remaining != 0u || state->gap_pending != 0u) {
        if (state->pending_requests != UINT8_MAX) {
            ++state->pending_requests;
        }
        return 0u;
    }
    state->duration_ticks = duration_ticks;
    state->ticks_remaining = duration_ticks;
    return 1u;
}

static inline uint8_t signal_state_tick(SignalState *state)
{
    if (state->gap_pending != 0u) {
        state->gap_pending = 0u;
        if (state->pending_requests != 0u) {
            --state->pending_requests;
            state->ticks_remaining = state->duration_ticks;
            return SIGNAL_TICK_STARTED;
        }
        return SIGNAL_TICK_NONE;
    }
    if (state->ticks_remaining == 0u) {
        return SIGNAL_TICK_NONE;
    }
    --state->ticks_remaining;
    if (state->ticks_remaining != 0u) {
        return SIGNAL_TICK_NONE;
    }
    state->gap_pending = 1u;
    return SIGNAL_TICK_FINISHED;
}

static inline uint8_t signal_state_is_active(const SignalState *state)
{
    return state->ticks_remaining != 0u;
}

static inline uint8_t signal_state_is_idle(const SignalState *state)
{
    return state->ticks_remaining == 0u &&
        state->pending_requests == 0u && state->gap_pending == 0u;
}

#endif
