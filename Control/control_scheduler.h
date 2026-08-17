#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include <stdint.h>

typedef struct {
    uint8_t brake_requested;
    uint8_t command_valid;
    int motor1;
    int motor2;
    int motor3;
    int motor4;
} ControlSchedulerOutput;

static inline void control_scheduler_init(ControlSchedulerOutput *output)
{
    output->brake_requested = 0u;
    output->command_valid = 0u;
    output->motor1 = 0;
    output->motor2 = 0;
    output->motor3 = 0;
    output->motor4 = 0;
}

static inline void control_scheduler_request_load(
    ControlSchedulerOutput *output, int motor1, int motor2, int motor3, int motor4)
{
    if (!output->brake_requested) {
        output->command_valid = 1u;
        output->motor1 = motor1;
        output->motor2 = motor2;
        output->motor3 = motor3;
        output->motor4 = motor4;
    }
}

static inline void control_scheduler_request_brake(ControlSchedulerOutput *output)
{
    output->brake_requested = 1u;
    output->command_valid = 0u;
}

static inline uint8_t control_scheduler_should_brake(
    const ControlSchedulerOutput *output)
{
    return (uint8_t)(output->brake_requested || !output->command_valid);
}

#endif
