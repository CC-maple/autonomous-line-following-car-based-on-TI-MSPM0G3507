/*
 由于GPIOB外部中断函数名只有一个，所以switch模式写在encoder.c中
 本工程oled源自平衡车oled
 printf串口重定向在board.c，由于已去掉MPU6050，因此串口0 printf 改回用pa10/pa11 可以直接XDS-USB连串口
 本工程延时函数未知（已将board.c中延时注释），oled中的延时作用域仅在oled文件】、
 所用陀螺仪为自己写的协议，在UART_gyro中，引脚与PCB相对应
 加上了delay函数
 
//8月1日晚上21:35备份

 */
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "oled.h"
#include "board.h"
#include "motor.h"
#include "control.h"
#include "encoder.h"
#include "uart_gyro.h"

int16_t nums;
char nums_buf[20];

int main(void)
{
    SYSCFG_DL_init();

    uart_gyro_init(); 

    Control_Init();
    Encoder_Init();//包含按键和编码器读取的外部中断

    // NVIC_EnableIRQ(UART0_INT_IRQn);
    // printf("printf UART0 OK!");

    oled_init();
    oled_show_string(0,0,"OLED OK!");
    OLED_CLS();
    static uint8_t j;
    while(1) 
    {
            oled_show_number_f1(0, 0, uart_gyro_heading_degrees());
            // oled_show_number_f1(0, 1, speed_left);
            // oled_show_number_f1(0, 2, speed_right);
            // oled_show_number_f1(0, 3, EncoderA);
            // oled_show_number_f1(0, 4, EncoderB);
            oled_show_number_f1(40, 0, Position_KP);
            oled_show_number_f1(70, 0, mode3_1_angle4_change);
            // oled_show_number_f1(90, 0, mode3_1_angle4_change);

            
            oled_show_number_f1(0, 1, huidu_data[1]);
            oled_show_number_f1(8, 1, huidu_data[2]);
            oled_show_number_f1(16, 1, huidu_data[3]);
            oled_show_number_f1(32, 1, huidu_data[4]);
            oled_show_number_f1(40, 1, huidu_data[5]);
            oled_show_number_f1(48, 1, huidu_data[6]);
            oled_show_string(0, 2, "mode:");
            oled_show_number_f1(31, 2, mode);
            oled_show_number_f1(50, 2, mode3_1);
            // oled_show_number_f1(0, 4, speed_left);
            // oled_show_number_f1(31, 4, speed_right);//前三问的速度值 已经调了 不用再显示了

            oled_show_string(0, 6, "mode4:"); oled_show_number_f1(32,6, mode4_flag); oled_show_number_f1(48,6,(int)(100*mode4_angle_change));
            oled_show_number_f1(0, 7, mode4_huidu_data[0]);
            oled_show_number_f1(8, 7, mode4_huidu_data[1]);
            oled_show_number_f1(16, 7, mode4_huidu_data[2]);
            oled_show_number_f1(24, 7, mode4_huidu_data[3]);
            oled_show_number_f1(40, 7, mode4_huidu_data[4]);
            oled_show_number_f1(48, 7, mode4_huidu_data[5]);
            oled_show_number_f1(56, 7, mode4_huidu_data[6]);
            oled_show_number_f1(62, 7, mode4_huidu_data[7]);
            
            // printf("%.2f",EncoderB);
    }
}




