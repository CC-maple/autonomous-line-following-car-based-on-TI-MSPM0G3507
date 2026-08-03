#include "uart_gyro.h"
#include "ti_msp_dl_config.h"
#include "encoder.h"

uint8_t gEchoData = 0;
uint8_t data_tx[15];

double Angle[3], T;

// char Angle_buf[20];

void uart_gyro_init()
{
    NVIC_ClearPendingIRQ(UART_gyro_INST_INT_IRQN);//
    NVIC_EnableIRQ(UART_gyro_INST_INT_IRQN);
}

void uart_gyro_disabled()
{
    NVIC_DisableIRQ(UART_gyro_INST_INT_IRQN);
}

void DecodeIMUData(uint8_t chrT[])
{
    Angle[0]= ( (short) (chrT[3]<<8 | chrT[2]))/32768.0*180;
    Angle[1]= ((short)(chrT[5]<<8|chrT[4]))/32768.0*180;
    Angle[2]=((short)(chrT[7]<<8|chrT[6]))/32768.0*180;
    T=((short)(chrT[9]<<8|chrT[8]))/340.0+36.25;
}

void UART_gyro_INST_IRQHandler(void)
{
    static uint8_t data_flag=0;
    static uint8_t i=0; 
    switch (DL_UART_Main_getPendingInterrupt(UART_gyro_INST)) {
        case DL_UART_MAIN_IIDX_RX://判断为接收中断
            gEchoData = DL_UART_Main_receiveData(UART_gyro_INST);
            if (data_flag==0) {
                if (gEchoData == 0x55){
                    data_tx[i]=gEchoData;
                    i+=1;
                    data_flag = 1;
                }
            }
            else if (data_flag==1) {
                if (gEchoData == 0x53) {
                    data_tx[i]=gEchoData;
                    i+=1;
                    data_flag = 2;
                }
            }
            else if(data_flag == 2){
                if (i<=9) {
                    data_tx[i]=gEchoData;
                    i++;//一共八个数据
                }
                else {
                    i=0;
                    data_flag = 0;
                    DecodeIMUData(data_tx);
                    // Encoder_Init();
                    //DL_UART_Main_transmitData(UART_1_INST, Angle[2]);
                }
            }
            break;
        default:
            break;
    }
}
