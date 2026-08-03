#include "control.h"
// #include  "math.h"
#include  "encoder.h"
#include "motor.h"
#include "oled.h"
#include <ti/driverlib/dl_timerg.h>
#include "ti_msp_dl_config.h"
#include "uart_gyro.h"


int64_t EncoderA, EncoderB, EncoderC,EncoderD;

int16_t speed_left=15,speed_right=15,speed_const=15;
int16_t angle_speed_left=0,angle_speed_right=0,angle_speed_const=0;

float Position_KP=3.0,Position_KI=0,Position_KD=0;//不能改，影响第一问 第二问 这是走直线的PID
float Angle_KP=0.010,Angle_KI=0,Angle_KD=0;//不能随便乱改，影响第二问 这是角度调整的pid

float PathLine_KP=0.005,PathLine_KI=0,PathLine_KD=0;//这是走固定长度路径的pid

float mode3_Angle_KP=0.1,mode3_Angle_KI=0,mode3_Angle_KD=0;//不能随便乱改，影响第二问 这是角度调整的pid

uint8_t mode2_1=0,mode2_2=0;//第二问分情况
uint8_t mode3_1=0, mode3_2=0;
int Position_target=0, Angle_target=0;
int path_line=0;

int pid_ans=0;
int pid_pathline_ans=0;
int pid_angle=0;
int test;
int32_t huidu_read_status=1;
int8_t huidu_data[8];
int8_t huidu_data_sum;

#define MODE3_1_Line1 2000
#define MODE3_1_Angle1 -90
#define MODE3_1_Line2 3500
#define MODE3_1_Angle2 0



uint8_t i;
#define Integral_bias_MAX 5
#define Integral_bias_MIN 1
/**
  * @brief  系统控制初始化(使用定时器TIMG0)
  * @param  无
  * @retval 无
  */
void Control_Init(void)
{
    NVIC_EnableIRQ(TIMG0_INT_IRQn);//使能中断
    DL_TimerG_startCounter(TIMG0);
}

int Position_PID(int reality,int target)//走直线的PID函数，勿动
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=reality-target;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> Integral_bias_MAX) Integral_bias = Integral_bias_MAX;   /* 积分限幅 */
    if(Integral_bias<-Integral_bias_MAX) Integral_bias = -Integral_bias_MIN;
    
    Pwm = (Position_KP*Bias)                        /* 比例环节 */
         +(Position_KI*Integral_bias)               /* 积分环节 */
         +(Position_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}

int Angle_PID(int reality,int target)//转一定角度的PID函数，有待优化
{ 	
    static float Bias_Angle,Pwm,Last_Bias_Angle,Integral_bias_Angle=0;
    
    Bias_Angle = reality-target;                            /* 计算偏差 */
    Integral_bias_Angle += Bias_Angle;	                        /* 偏差累积 */
    
    if(Integral_bias_Angle> Integral_bias_MAX) Integral_bias_Angle = Integral_bias_MAX;   /* 积分限幅 */
    if(Integral_bias_Angle<-Integral_bias_MAX) Integral_bias_Angle = -Integral_bias_MIN;
    
    Pwm = (Angle_KP*Bias_Angle)                        /* 比例环节 */
         +(Angle_KI*Integral_bias_Angle)               /* 积分环节 */
         +(Angle_KD*(Bias_Angle-Last_Bias_Angle));           /* 微分环节 */
    
    Last_Bias_Angle=Bias_Angle;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}

int mode3_Angle_PID(int reality,int target)//转一定角度的PID函数，有待优化
{ 	
    static float Bias_Angle,Pwm,Last_Bias_Angle,Integral_bias_Angle=0;
    
    Bias_Angle = reality-target;                            /* 计算偏差 */
    Integral_bias_Angle += Bias_Angle;	                        /* 偏差累积 */
    
    if(Integral_bias_Angle> Integral_bias_MAX) Integral_bias_Angle = Integral_bias_MAX;   /* 积分限幅 */
    if(Integral_bias_Angle<-Integral_bias_MAX) Integral_bias_Angle = -Integral_bias_MIN;
    
    Pwm = (mode3_Angle_KP*Bias_Angle)                        /* 比例环节 */
         +(mode3_Angle_KI*Integral_bias_Angle)               /* 积分环节 */
         +(mode3_Angle_KD*(Bias_Angle-Last_Bias_Angle));           /* 微分环节 */
    
    Last_Bias_Angle=Bias_Angle;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}

int PathLine_PID(int reality,int target)//走直线的PID函数，勿动
{ 	
    static float Bias_PathLine,Pwm,Last_Bias_PathLine,Integral_bias_PathLine=0;
    
    Bias_PathLine=target-reality;                            /* 计算偏差 目标-实际*/
    Integral_bias_PathLine+=Bias_PathLine;	                        /* 偏差累积 */
    
    if(Integral_bias_PathLine> Integral_bias_MAX) Integral_bias_PathLine = Integral_bias_MAX;   /* 积分限幅 */
    if(Integral_bias_PathLine<-Integral_bias_MAX) Integral_bias_PathLine = -Integral_bias_MIN;

    Pwm = (PathLine_KP*Bias_PathLine)                        /* 比例环节 */
         +(PathLine_KI*Integral_bias_PathLine)               /* 积分环节 */
         +(PathLine_KD*(Bias_PathLine-Last_Bias_PathLine));           /* 微分环节 */
    
    Last_Bias_PathLine=Bias_PathLine;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}



//正数值限幅函数 用于限制速度 输入速度变量和最大最小值
int limit_control(int x,int x_min, int x_max)
{
    if (x > x_max) x=x_max;
    else if (x < x_min) x=x_min;
    return x;
}

void huidu_updata()
{
    huidu_read_status = DL_GPIO_readPins( GPIO_Sensor_PORT,
                GPIO_Sensor_PIN_huidu6_PIN | GPIO_Sensor_PIN_huidu5_PIN | GPIO_Sensor_PIN_huidu4_PIN |
                GPIO_Sensor_PIN_huidu3_PIN| GPIO_Sensor_PIN_huidu2_PIN | GPIO_Sensor_PIN_huidu1_PIN);
    huidu_data_sum=0;//求和前清零
    for (i=1;i<=6;i++) {
        huidu_data_sum+=huidu_data[i];//更新求和
    }

    if ((huidu_read_status & GPIO_Sensor_PIN_huidu1_PIN)) {
    huidu_data[1]=1;
    }
    else {
    huidu_data[1]=0;
    }

    if (huidu_read_status & GPIO_Sensor_PIN_huidu2_PIN) {
        huidu_data[2]=1;
    }
    else {
        huidu_data[2]=0;
    }

    if (huidu_read_status & GPIO_Sensor_PIN_huidu3_PIN) {
        huidu_data[3]=1;
    }
    else {
        huidu_data[3]=0;
    }

    if (huidu_read_status & GPIO_Sensor_PIN_huidu4_PIN) {
        huidu_data[4]=1;
        }
    else {
    huidu_data[4]=0;
    }

    if (huidu_read_status & GPIO_Sensor_PIN_huidu5_PIN) {
    huidu_data[5]=1;
    }
    else {
    huidu_data[5]=0;
    }

    if (huidu_read_status & GPIO_Sensor_PIN_huidu6_PIN) {
    huidu_data[6]=1;
    }
    else {
    huidu_data[6]=0;
    }
    
}

void mode3_go_line(int mode3_target_line, float mode3_line_angle){
    encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
    path_line = path_line+ my_abs(EncoderA)+ my_abs(EncoderB);
    pid_ans=Position_PID(Angle[2], mode3_line_angle);
    speed_left+=pid_ans;
    speed_right-=pid_ans;

    speed_left=limit_control(speed_left, 10, 30);
    speed_right=limit_control(speed_right, 10, 30);
    
    if ( my_abs(mode3_target_line-path_line) <= 30 ) {
        path_line=0;
        if (mode3_target_line==MODE3_1_Line1) {
            brake();
            speed_left=0;
            speed_right=0;
            mode3_1=1;
        }
        else if (mode3_target_line==MODE3_1_Line2) {
            brake();
            speed_left=0;
            speed_right=0;
            mode3_1=3;
        }
        // mode3_1=1;模式切换 要返回值
    }
    else {
        Load(speed_left, speed_right, speed_const, speed_const);
    }
}


void mode3_only_go_line(float mode3_line_angle){
    huidu_updata();//更新灰度
    pid_ans=Position_PID(Angle[2],Position_target);
    speed_left+=pid_ans;
    speed_right-=pid_ans;
    speed_left = limit_control(speed_left, 10, 30);
    speed_right = limit_control(speed_right, 10, 30);
    huidu_updata();
    if (huidu_data_sum!=6) {//退出条件：遇到黑线
        brake();
        mode=5;//这里要补充切换模式
    }
    else {
        Load(speed_left, speed_right, speed_const, speed_const);
        encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
    }
}


void mode3_turn_angle(float mode3_target_angle)
{
    static float test;
    pid_angle=mode3_Angle_PID(Angle[2], mode3_target_angle);
    speed_left+=pid_angle;
    speed_right-=pid_angle;
    speed_left = limit_control(speed_left, -30, 30);
    speed_right = limit_control(speed_right, -30, 30);
    Load(speed_left, speed_right, speed_left,speed_right);
    test=get_abs(  (int)(Angle[2])  -mode3_target_angle );
    if ( test <= 1) {
        if (mode3_target_angle == MODE3_1_Angle1) {//根据模式不同选择跳转不同的模式
            mode3_1 = 2;
        }
        else if (mode3_target_angle==MODE3_1_Angle2) {
            mode3_1 = 4;
        }
    }                                                                                                                          
}
/**
  * @brief  系统控制中断
  * @param  无
  * @retval 无
  */
void TIMG0_IRQHandler(void)
{
    int PWM_out;int i;
    switch(DL_TimerG_getPendingInterrupt(TIMG0))
	{
		case DL_TIMER_IIDX_ZERO:
		{        
			// NVIC_DianableIRQ(GPIOB_INT_IRQn);//包含按键和编码器读取的外部中断,
            if (mode==1) {//模式1 走直线
                huidu_updata();//更新灰度

                pid_ans=Position_PID(Angle[2],Position_target);
                // speed_left=speed_const;
                // speed_right=speed_const;
                speed_left+=pid_ans;
                speed_right-=pid_ans;
                speed_left = limit_control(speed_left, 10, 30);
                speed_right = limit_control(speed_right, 10, 30);
                if (huidu_data_sum!=6) {
                    brake();
                    mode=5;
                }
                else {
                    Load(speed_left, speed_right, speed_const, speed_const);
                    encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                }
            }
            else if (mode == 2) {
                // huidu_read_status = DL_GPIO_readPins(GPIO_Sensor_PORT, GPIO_Sensor_PIN_huidu2_PIN);
                huidu_updata();
                //走直线A------B 目标角度 0度
                if (mode2_1==0 && mode2_2==0) {
                    pid_ans=Position_PID(Angle[2],0);
                    speed_left+=pid_ans;
                    speed_right-=pid_ans;
                    speed_left = limit_control(speed_left, 10, 30);
                    speed_right = limit_control(speed_right, 10, 30);
                    if (huidu_data_sum!=6) {
                        brake();
                        mode2_1=1;
                        mode2_2=0;
                    }
                    else {
                        Load(speed_left, speed_right, speed_const, speed_const);
                        encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                    }
                }
                //----------------------------------------------------
                //巡线B------C 
                else if (mode2_1==1&&mode2_2==0) {
                    if (huidu_data[1]==0) {
                    speed_left = 0.8*speed_const;
                    //speed_left -= 5;
                    speed_right = 4.1*speed_const;
                    }
                    else if (huidu_data[6]==0) {
                        speed_left = 4.1*speed_const;
                        // speed_right -= 5;
                        speed_right  = 0.8*speed_const ;
                    }

                    else if (huidu_data[2]==0&&huidu_data[3]==1) {
                    // speed_left  -=3;
                    // speed_right +=3;
                        speed_left = 0.5*speed_const ;
                        speed_right = 3.1*speed_const;
                    }

                    else if (huidu_data[2]==0&&huidu_data[3]==0) {
                        // speed_left = speed_const;
                    // speed_left -=3 ;
                        //speed_right += 3;
                        speed_left = 0.5*speed_const ;
                        speed_right = 2.6*speed_const;
                    }

                    else if (huidu_data[3]==0&&huidu_data[4]==0) {
                        speed_left = 2*speed_const;
                        speed_right = 0.8*speed_const;
                    }
                    else if (huidu_data[4]==0&&huidu_data[5]==0) {
                        speed_left = 2.6*speed_const ;
                        speed_right = 0.5*speed_const;
                    }

                    else if (huidu_data[4]==1 && huidu_data[5]==0) {
                        // speed_left = 0.8*speed_const;
                        //speed_left+=3;
                        //speed_right -=3;
                        speed_left = 3.1*speed_const ;
                        speed_right = 0.5*speed_const;
                    }
                    
                
                    else if(huidu_data_sum==6){//巡线到达白色区域，开始调整角度
                        //调整角度 -180度为目标角度
                        pid_angle=Angle_PID(Angle[2], -179);
                        speed_left+=pid_angle;
                        speed_right-=pid_angle;
                        speed_left = limit_control(speed_left, -30, 30);
                        speed_right = limit_control(speed_right, -30, 30);
                        Load(speed_left, speed_right, speed_left,speed_right);
                        if (my_abs(Angle[2]+179)<2) {
                            mode2_1=1,mode2_2=1;
                        }
                    }
                               
                speed_left=limit_control(speed_left, 10, 65);
                speed_right=limit_control(speed_right, 5, 60);
                Load(speed_left,speed_right,speed_left,speed_right);
                encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                }


                //走直线 C-----D
                else if (mode2_1==1&&mode2_2==1) {
                    pid_ans=Position_PID(Angle[2],-178);//第二次走直线目标角度  （）
                    speed_left+=pid_ans;
                    speed_right-=pid_ans;
                    speed_left = limit_control(speed_left, 10, 30);
                    speed_right = limit_control(speed_right, 10, 30);
                    if (huidu_data_sum!=6) {
                        brake();
                        mode2_1=0;
                        mode2_2=1;
                    }
                    else {
                        Load(speed_left, speed_right, speed_const, speed_const);
                        encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                    }
                }
                //巡线D------A
                if (mode2_1==0&&mode2_2==1) {
                    if (huidu_data[1]==0) {
                    speed_left = 0.8*speed_const;
                    //speed_left -= 5;
                    speed_right = 4.1*speed_const;
                    }
                    else if (huidu_data[6]==0) {
                        speed_left = 4.1*speed_const;
                        // speed_right -= 5;
                        speed_right  = 0.8*speed_const ;
                    }

                    else if (huidu_data[2]==0&&huidu_data[3]==1) {
                    // speed_left  -=3;
                    // speed_right +=3;
                        speed_left = 0.5*speed_const ;
                        speed_right = 3.1*speed_const;
                    }

                    else if (huidu_data[2]==0&&huidu_data[3]==0) {
                        // speed_left = speed_const;
                    // speed_left -=3 ;
                        //speed_right += 3;
                        speed_left = 0.5*speed_const ;
                        speed_right = 2.6*speed_const;
                    }

                    else if (huidu_data[3]==0&&huidu_data[4]==0) {
                        speed_left = 2*speed_const;
                        speed_right = 0.8*speed_const;
                    }
                    else if (huidu_data[4]==0&&huidu_data[5]==0) {
                        speed_left = 2.6*speed_const ;
                        speed_right = 0.5*speed_const;
                    }

                    else if (huidu_data[4]==1 && huidu_data[5]==0) {
                        // speed_left = 0.8*speed_const;
                        //speed_left+=3;
                        //speed_right -=3;
                        speed_left = 3.1*speed_const ;
                        speed_right = 0.5*speed_const;
                    }
                    else if(huidu_data_sum==6){//巡线退出条件
                        mode=5;
                        mode2_1=0;mode2_2=0;
                    }
                    speed_left=limit_control(speed_left, 10, 65);
                    speed_right=limit_control(speed_right, 5, 60);
                    Load(speed_left,speed_right,speed_left,speed_right);
                    encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                }






                // else if(huidu_data_sum==6){//调整角度
                    
                //     Position_target = Angle[2];//切换目标值
                //     pid_angle=Angle_PID(Angle[2], -179);
                //     // pid_angle=Angle_PID(Angle[2], -90);
                //     speed_left+=pid_angle;
                //     speed_right-=pid_angle;
                //     speed_left = limit_control(speed_left, -30, 30);
                //     speed_right = limit_control(speed_right, -30, 30);
                //     Load(speed_left, speed_right, speed_left,speed_right);
                    
                //     if (my_abs(Angle[2]+179)<2) {
                //         mode=1;
                //     }
                // }
                
            }

            else if (mode == 3) {
                if(mode3_1==0)
                {
                //    mode3_go_line(MODE3_1_Line1,MODE3_1_Angle1);
                mode3_go_line(MODE3_1_Line1,0);
                }
                else if(mode3_1==1){
                    mode3_turn_angle(MODE3_1_Angle1);
                }
                else if (mode3_1==2) {
                    mode3_go_line(MODE3_1_Line2, MODE3_1_Angle1);//对
                }
                else if (mode3_1==3) {
                    mode3_turn_angle(MODE3_1_Angle2);
                }
                else if (mode3_1==4) {
                    mode3_only_go_line(0);
                }

            }
            else if (mode == 4) {
                brake();
                mode=5;
            }
            else if(mode == 5)
            {
                brake();
                huidu_updata();
            }
            if (mode == 0) {
                Load(speed_left, speed_right, speed_left, speed_right);
                encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
            }

		}

			break;

			default:
				break;
			}
}

