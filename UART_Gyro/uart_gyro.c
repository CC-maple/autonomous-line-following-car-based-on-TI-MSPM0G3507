#include "uart_gyro.h"
#include "imu_protocol.h"
#include "ti_msp_dl_config.h"

uint8_t gEchoData = 0;
uint8_t data_tx[15];

double Angle[3], T;

static ImuFrameParser imu_parser;
static volatile int16_t imu_heading_cdeg;
static volatile uint16_t imu_sample_age_ticks = IMU_SAMPLE_AGE_INVALID;

static void uart_gyro_reset_parser_and_invalidate(void)
{
    imu_frame_parser_reset(&imu_parser);
    imu_sample_invalidate(&imu_sample_age_ticks);
}

void uart_gyro_init(void)
{
    uart_gyro_reset_parser_and_invalidate();
    NVIC_ClearPendingIRQ(UART_gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_gyro_INST_INT_IRQN);
}

void uart_gyro_disabled(void)
{
    NVIC_DisableIRQ(UART_gyro_INST_INT_IRQN);
}

void DecodeIMUData(const uint8_t chrT[])
{
    Angle[0] = ((int16_t)(((uint16_t)chrT[3] << 8) | chrT[2])) / 32768.0 * 180;
    Angle[1] = ((int16_t)(((uint16_t)chrT[5] << 8) | chrT[4])) / 32768.0 * 180;
    Angle[2] = ((int16_t)(((uint16_t)chrT[7] << 8) | chrT[6])) / 32768.0 * 180;
    T = ((int16_t)(((uint16_t)chrT[9] << 8) | chrT[8])) / 340.0 + 36.25;
}

void uart_gyro_tick(void)
{
    imu_sample_tick(&imu_sample_age_ticks);
}

uint8_t uart_gyro_is_fresh(void)
{
    return imu_sample_is_fresh(&imu_sample_age_ticks);
}

float uart_gyro_heading_degrees(void)
{
    return (float)imu_heading_cdeg / 100.0f;
}

void UART_gyro_INST_IRQHandler(void)
{
    const uint8_t *accepted_frame;
    uint8_t parser_result;

    switch (DL_UART_Main_getPendingInterrupt(UART_gyro_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            if (DL_UART_Main_getErrorStatus(UART_gyro_INST,
                DL_UART_MAIN_ERROR_OVERRUN | DL_UART_MAIN_ERROR_BREAK |
                DL_UART_MAIN_ERROR_PARITY | DL_UART_MAIN_ERROR_FRAMING) != 0u) {
                gEchoData = DL_UART_Main_receiveData(UART_gyro_INST);
                uart_gyro_reset_parser_and_invalidate();
                break;
            }

            gEchoData = DL_UART_Main_receiveData(UART_gyro_INST);
            parser_result = imu_frame_parser_push(
                &imu_parser, gEchoData, &accepted_frame);
            if (parser_result == IMU_FRAME_ACCEPTED) {
                DecodeIMUData(accepted_frame);
                imu_heading_cdeg = imu_frame_heading_cdeg(accepted_frame);
                imu_sample_publish(&imu_sample_age_ticks);
            }
            else if (parser_result == IMU_FRAME_CHECKSUM_FAILED) {
                imu_sample_invalidate(&imu_sample_age_ticks);
            }
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            DL_UART_Main_clearInterruptStatus(
                UART_gyro_INST,
                DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
            uart_gyro_reset_parser_and_invalidate();
            break;

        default:
            break;
    }
}
