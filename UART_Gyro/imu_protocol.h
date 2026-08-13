#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H

#include <stdint.h>

#define IMU_FRAME_LENGTH 11u
#define IMU_FRAME_HEADER 0x55u
#define IMU_FRAME_ANGLE_TYPE 0x53u
#define IMU_FRAME_NO_RESULT 0u
#define IMU_FRAME_ACCEPTED 1u
#define IMU_FRAME_CHECKSUM_FAILED 2u
#define IMU_SAMPLE_MAX_AGE_TICKS 10u
#define IMU_SAMPLE_AGE_INVALID UINT16_MAX

static inline void imu_sample_invalidate(volatile uint16_t *age_ticks)
{
    *age_ticks = IMU_SAMPLE_AGE_INVALID;
}

static inline void imu_sample_publish(volatile uint16_t *age_ticks)
{
    *age_ticks = 0u;
}

static inline void imu_sample_tick(volatile uint16_t *age_ticks)
{
    if (*age_ticks != IMU_SAMPLE_AGE_INVALID &&
        *age_ticks < (uint16_t)(IMU_SAMPLE_AGE_INVALID - 1u)) {
        ++(*age_ticks);
    }
}

static inline uint8_t imu_sample_is_fresh(volatile const uint16_t *age_ticks)
{
    return *age_ticks <= IMU_SAMPLE_MAX_AGE_TICKS;
}

typedef struct {
    uint8_t bytes[IMU_FRAME_LENGTH];
    uint8_t index;
} ImuFrameParser;

static inline void imu_frame_parser_reset(ImuFrameParser *parser)
{
    parser->index = 0u;
}

static inline uint8_t imu_frame_checksum_is_valid(const uint8_t *frame)
{
    uint8_t index;
    uint8_t checksum = 0u;

    for (index = 0u; index < IMU_FRAME_LENGTH - 1u; ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum == frame[IMU_FRAME_LENGTH - 1u];
}

static inline int16_t imu_frame_heading_cdeg(const uint8_t *frame)
{
    int16_t raw_heading = (int16_t)(((uint16_t)frame[7] << 8) | frame[6]);

    return (int16_t)(((int32_t)raw_heading * 18000) / 32768);
}

static inline uint8_t imu_frame_parser_push(
    ImuFrameParser *parser, uint8_t byte, const uint8_t **accepted_frame)
{
    if (parser->index == 0u) {
        if (byte == IMU_FRAME_HEADER) {
            parser->bytes[0] = byte;
            parser->index = 1u;
        }
        return IMU_FRAME_NO_RESULT;
    }

    if (parser->index == 1u) {
        if (byte == IMU_FRAME_ANGLE_TYPE) {
            parser->bytes[1] = byte;
            parser->index = 2u;
        }
        else if (byte == IMU_FRAME_HEADER) {
            parser->bytes[0] = byte;
        }
        else {
            parser->index = 0u;
        }
        return IMU_FRAME_NO_RESULT;
    }

    parser->bytes[parser->index] = byte;
    ++parser->index;
    if (parser->index < IMU_FRAME_LENGTH) {
        return IMU_FRAME_NO_RESULT;
    }

    if (imu_frame_checksum_is_valid(parser->bytes)) {
        *accepted_frame = parser->bytes;
        parser->index = 0u;
        return IMU_FRAME_ACCEPTED;
    }

    if (byte == IMU_FRAME_HEADER) {
        parser->bytes[0] = byte;
        parser->index = 1u;
    }
    else {
        parser->index = 0u;
    }
    return IMU_FRAME_CHECKSUM_FAILED;
}

#endif
