#include "control.h"
// #include  "math.h"
#include  "encoder.h"
#include "motor.h"
#include "control_math.h"
#include "oled.h"
// #include <cstdint>
#include <ti/driverlib/dl_timerg.h>
#include "ti_msp_dl_config.h"
#include "uart_gyro.h"

#include "math.h"

int64_t EncoderA, EncoderB, EncoderC,EncoderD;

int16_t speed_left=15,speed_right=15,speed_const=15;
int16_t angle_speed_left=0,angle_speed_right=0,angle_speed_const=0;

float Position_KP=3.0,Position_KI=0,Position_KD=0;//不能改，影响第一问 第二问 这是走直线的PID
float Angle_KP=0.010,Angle_KI=0,Angle_KD=0;//不能随便乱改，影响第二问 这是角度调整的pid

float PathLine_KP=0.005,PathLine_KI=0,PathLine_KD=0;//这是走固定长度路径的pid

float mode3_Angle_KP=0.1,mode3_Angle_KI=0,mode3_Angle_KD=0;//不能随便乱改，影响第二问 这是角度调整的pid

uint8_t mode2_1=0,mode2_2=0;//第二问分情况
uint8_t mode2_bee=0;

uint8_t mode3_1=0, mode3_2=0;
// uint8_t mode4_num=0;//

int Position_target=0, Angle_target=0;
uint64_t path_line=0;

int pid_ans=0;
int pid_pathline_ans=0;
int pid_angle=0;
int32_t huidu_read_status=1;
int8_t huidu_data[8];
int8_t huidu_data_sum;


//--------------------mode4变量专用--------------------------------------------//
int Mode4_line_PID(float reality_angle, float target_angle);
//对浮点数用绝对值函数会导致精度丢失浮点部分
float Mode4_line_Kp=0.3, Mode4_line_Kd=0.08;
int mode4_V_C=5, mode4_pwm_ans; //模式4的左速度、右速度、速度常数

int mode4_v_L, mode4_v_R;
uint8_t mode4_flag=0; //模式4的细分模式
uint8_t mode4_color_flag;//color=0 多次识别到黑线，否则：color=1(处于白色区)
uint8_t mode4_color_nums;//颜色记数，防止误检测，成功记数后记得清零

void mode4_huidu_updata();
int32_t mode4_huidu_read_status=1;
int8_t mode4_huidu_data[8];
int8_t mode4_huidu_data_sum;

float mode4_angle1_1=0;//第一圈的微调角度变量1
uint8_t mode4_angle_flag;//微调角度更新标志位 1：更新完毕， 0：未更新
float mode4_angle_change=-145;//按键更改第一圈第二次斜线角度变量 //经测试145.4 或145.6效果很好

uint8_t mode4_circle_nums=0;
//-------------------------------------------------------------------------//

#define MODE3_DISTANCE_TOLERANCE 15u
#define MODE3_1_Line1 2000
#define MODE3_1_Angle1 -90
#define MODE3_1_Line2 3630//+30+30 //原3500
#define MODE3_1_Angle2 -4

#define MODE3_1_Line3 2001
#define MODE3_1_Angle3 175

#define  MODE3_1_Angle4 -96
#define MODE3_1_Line4 4500
uint16_t mode3_1_angle4_change = MODE3_1_Line4;
#define MODE3_1_Angle5 -179

uint8_t i;
#define Integral_bias_MAX 5
#define Integral_bias_MIN 1

void Sign_LED_Bee(){
    DL_GPIO_clearPins(GPIO_Sign_PORT,GPIO_Sign_PIN_LED_PIN | GPIO_Sign_PIN_Bee_PIN);
    delay_cycles(16000000);
    DL_GPIO_setPins(GPIO_Sign_PORT,GPIO_Sign_PIN_LED_PIN | GPIO_Sign_PIN_Bee_PIN);
}

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
    huidu_read_status = DL_GPIO_readPins( GPIOA,
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

void mode3_go_line(uint64_t mode3_target_line, float mode3_line_angle){
    uint64_t encoder_step;

    encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
    encoder_step = control_add_u64_saturating(
        control_abs_i64(EncoderA), control_abs_i64(EncoderB));
    path_line = control_add_u64_saturating(path_line, encoder_step);
    pid_ans=Position_PID(Angle[2], mode3_line_angle);
    speed_left+=pid_ans;
    speed_right-=pid_ans;

    speed_left=limit_control(speed_left, 10, 30);
    speed_right=limit_control(speed_right, 10, 30);
    
    if (control_distance_reached(path_line, mode3_target_line, MODE3_DISTANCE_TOLERANCE)) {
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
        else if (mode3_target_line == MODE3_1_Line3) {
            brake();
            speed_left=0;
            speed_right=0;
            mode3_1=7;
        }
        else if (mode3_target_line == mode3_1_angle4_change) {
            brake();
            speed_left=0;
            speed_right=0;
            mode3_1=9;
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
        mode3_1 += 1;//这里 已经 补充切换模式
        Sign_LED_Bee();
    }
    else {
        Load(speed_left, speed_right, speed_const, speed_const);
        encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
    }
}


void mode3_turn_angle(float mode3_target_angle)
{
    uint64_t angle_error;
    // pid_angle=mode3_Angle_PID(Angle[2], mode3_target_angle);//!!!!!!!!!!
    ///-----------------------------------------------------------------!!!!!!!!!!!!!!-------------------------
    if (mode3_1==7) {
        if (Angle[2]>=0) {
            Angle[2] = (-1)*Angle[2];
        }
        pid_angle=mode3_Angle_PID(Angle[2], mode3_target_angle);
        speed_left+=pid_angle;
        speed_right-=pid_angle;
    }
    else {
        pid_angle=mode3_Angle_PID(Angle[2], mode3_target_angle);
        speed_left+=pid_angle;
        speed_right-=pid_angle;
    }
    speed_left = limit_control(speed_left, -30, 30);
    speed_right = limit_control(speed_right, -30, 30);
    angle_error = control_abs_diff_i64((int64_t)Angle[2], (int64_t)mode3_target_angle);

    if (angle_error <= 1u) {
        brake();
        if (mode3_target_angle == MODE3_1_Angle1) {//根据模式不同选择跳转不同的模式
            mode3_1 = 2;
        }
        else if (mode3_target_angle==MODE3_1_Angle2) {
            mode3_1 = 4;
        }
        else if (mode3_target_angle == MODE3_1_Angle4) {
            mode3_1 = 8;
        }
        else if (mode3_target_angle == MODE3_1_Angle5) {
            mode3_1=10;
        }
    }
    else {
        Load(speed_left, speed_right, speed_left,speed_right);
    }                                                                                                                          
}



void search_line()
{
}//搜索函数


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
        if(begin)
		{        
			// NVIC_DianableIRQ(GPIOB_INT_IRQn);//包含按键和编码器读取的外部中断,
            {       
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
                        Sign_LED_Bee();
                        mode=0;
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
                            //-----------//
                            Sign_LED_Bee();
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
                            if (mode2_bee==0) {
                                brake();
                                // //-----------//
                                Sign_LED_Bee();
                                mode2_bee=1;
                            }
                            else {
                                pid_angle=Angle_PID(Angle[2], -179);
                                speed_left+=pid_angle;
                                speed_right-=pid_angle;
                                speed_left = limit_control(speed_left, -30, 30);
                                speed_right = limit_control(speed_right, -30, 30);
                                if (  (control_abs_i64((int64_t)(Angle[2] + 179)) < 2u) ||
                                     (control_abs_i64((int64_t)(Angle[2] - 179)) < 2u)  ) {
                                    brake();
                                    mode2_1=1,mode2_2=1;
                                }
                                else {
                                    Load(speed_left, speed_right, speed_left,speed_right);
                                }
                            }

                        }
                                
                    if (huidu_data_sum!=6) {
                        speed_left=limit_control(speed_left, 10, 65);
                        speed_right=limit_control(speed_right, 5, 60);
                        Load(speed_left,speed_right,speed_left,speed_right);
                        encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                    }
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
                            Sign_LED_Bee();
                            //-----------//

                            mode2_1=0;
                            mode2_2=1;
                        }
                        else {
                            Load(speed_left, speed_right, speed_const, speed_const);
                            encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                        }
                    }
                    //巡线D------A
                    else if (mode2_1==0&&mode2_2==1) {
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
                            brake();
                            Sign_LED_Bee();
                            mode=0;
                            mode2_1=0;mode2_2=0;
                        }
                        if (huidu_data_sum!=6) {
                            speed_left=limit_control(speed_left, 10, 65);
                            speed_right=limit_control(speed_right, 5, 60);
                            Load(speed_left,speed_right,speed_left,speed_right);
                            encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                        }
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
                        mode3_only_go_line(MODE3_1_Angle2);//零度走直线
                    }
                    else if (mode3_1==5) {
                        //进入寻迹 先更新灰度
                        huidu_updata();
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
                        else if ( huidu_data_sum==6 ) {//退出巡线条件
                            //调整角度 -180度为目标角度
                            // pid_angle=Angle_PID(Angle[2], 179);//出线调整的目标角度
                            // pid_angle=mode3_Angle_PID(Angle[2], 175);//出线调整的目标角度//测试调整角度
                            pid_angle=mode3_Angle_PID(Angle[2], MODE3_1_Angle3);//出线调整的目标角度//测试调整角度


                            speed_left+=pid_angle;
                            speed_right-=pid_angle;
                            speed_left = limit_control(speed_left, -30, 30);
                            speed_right = limit_control(speed_right, -30, 30);
                            Load(speed_left, speed_right, speed_left,speed_right);
                            if (control_abs_diff_u64(control_abs_i64((int64_t)Angle[2]), MODE3_1_Angle3) < 1u) {

                                brake();
                                //-----------//
                                Sign_LED_Bee();
                                mode3_1=6;
                            }
                        }
                        if (huidu_data_sum!=6) {
                            speed_left=limit_control(speed_left, 10, 65);
                            speed_right=limit_control(speed_right, 5, 60);
                            Load(speed_left,speed_right,speed_left,speed_right);
                            encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                        }
                    }
                    else if (mode3_1==6) {
                        mode3_go_line(MODE3_1_Line3, MODE3_1_Angle3);
                        
                    }
                    else if (mode3_1==7) {
                    mode3_turn_angle(MODE3_1_Angle4);
                    }
                    else if (mode3_1==8) {
                        mode3_go_line(mode3_1_angle4_change, MODE3_1_Angle4);
                    }
                    else if(mode3_1==9){
                        // mode3_turn_angle(MODE3_1_Angle3);//测试angle输入值是否对应
                        mode3_turn_angle(MODE3_1_Angle5);//angle5对应 正确
                    }
                    else if (mode3_1==10) {
                        mode3_only_go_line(MODE3_1_Angle5);
                    }
                    else if(mode3_1==11)
                    {
                        //进入寻迹 先更新灰度
                        huidu_updata();
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
                        else if ( huidu_data_sum==6 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode3_1=12;
                            mode=0;
                        }
                        if (huidu_data_sum!=6) {
                            speed_left=limit_control(speed_left, 10, 65);
                            speed_right=limit_control(speed_right, 5, 60);
                            Load(speed_left,speed_right,speed_left,speed_right);
                            encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                        }
                    }

                }

                else if (mode==4) {
                    if(mode4_flag==0)
                    {
                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], -37);
                        mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=2;
                        }
                    }

                    else if (mode4_flag==2) {
                        mode4_huidu_updata();//更新灰度
                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -2*speed_const ;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=3;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                    else if (mode4_flag==3) {
                        mode4_huidu_updata();
                        if ( Angle[2]>= 0 ) {
                            mode4_v_L =  -2*mode4_V_C;
                            mode4_v_R =  2*mode4_V_C;//V_C是速度常数，可调
                            mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                            mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                            Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        }
                        else {
                            mode4_pwm_ans = Mode4_line_PID(Angle[2], mode4_angle_change);// mode4_angle_change默认-144度 变量可调
                            mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                            mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                            mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                            mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                            Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                            if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                if (mode4_angle_flag==0) {
                                    mode4_angle1_1=Angle[2];//获取微调角度到变量
                                    mode4_angle_flag=1;
                                }
                                mode4_pwm_ans = Mode4_line_PID(mode4_angle1_1, mode4_angle1_1+4);
                                mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                                mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                                mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                                mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                                if (fabs(Angle[2]-mode4_angle1_1)<=0.5) {
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=4;
                                }
                                else {
                                    Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);  
                                }
                                

                            }
                        }
                    }
                    else if (mode4_flag==4) {
                        mode4_huidu_updata();//更新灰度

                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -3*speed_const;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            
                            mode4_circle_nums+=1;
                            if (mode4_circle_nums>=4) {
                                mode4_flag=5;
                                mode=0;
                            }
                            else {
                                mode4_flag=5;
                                
                            }
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                    else if(mode4_flag==5)
                    {
                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], -33);
                        mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=2;
                        }

                    }


                }
                else if(mode==5){
                
                    if(mode4_flag==0)
                    {
                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], -37);
                        mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=2;
                        }
                    }

                    else if (mode4_flag==2) {
                        mode4_huidu_updata();//更新灰度
                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -2*speed_const ;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=3;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                    else if (mode4_flag==3) {
                        mode4_huidu_updata();
                        if ( Angle[2]>= 0 ) {
                            mode4_v_L =  -2*mode4_V_C;
                            mode4_v_R =  2*mode4_V_C;//V_C是速度常数，可调
                            mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                            mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                            Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        }
                        else {
                            mode4_pwm_ans = Mode4_line_PID(Angle[2], mode4_angle_change);
                            mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                            mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                            mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                            mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                            Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                            if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                brake();
                                Sign_LED_Bee();
                                mode4_flag=4;
                            }
                        }
                    }
                    else if (mode4_flag==4) {
                        mode4_huidu_updata();//更新灰度
                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -3*speed_const ;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=5;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }

                }

                else if(mode==6)
                {
                    if(mode4_flag==0)
                    {
                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], -37);
                        mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=2;
                        }
                    }

                    else if (mode4_flag==2) {
                        mode4_huidu_updata();//更新灰度
                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -2*speed_const ;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=3;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                    else if (mode4_flag==3) {
                        mode4_huidu_updata();
                        if ( Angle[2]>= 0 ) {
                            mode4_v_L =  -2*mode4_V_C;
                            mode4_v_R =  2*mode4_V_C;//V_C是速度常数，可调
                            mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                            mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                            Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        }
                        else {
                            mode4_pwm_ans = Mode4_line_PID(Angle[2], -145);
                            mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                            mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                            mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                            mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                            Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                            if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                brake();
                                Sign_LED_Bee();
                                mode4_flag=4;
                            }
                        }
                    }
                    else if (mode4_flag==4) {
                        mode4_huidu_updata();//更新灰度
                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -3*speed_const ;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=5;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                    else if(mode4_flag==5)
                    {
                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], -33);
                        mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=2;
                        }

                    }
                }
                else if (mode==7) {
                    if(mode4_flag==0){
                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], 0);
                        mode4_v_L =  2*mode4_V_C + mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - mode4_pwm_ans ;//V_C是速度常数，可调
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=2;
                        }
                    }

                    else if (mode4_flag==2) {
                        mode4_huidu_updata();//更新灰度
                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -2*speed_const ;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=3;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                    else if (mode4_flag==3) {

                        mode4_huidu_updata();
                        mode4_pwm_ans = Mode4_line_PID(Angle[2], -179);
                        mode4_v_L =  2*mode4_V_C + 0.5*mode4_pwm_ans ;
                        mode4_v_R =  2*mode4_V_C - 0.5*mode4_pwm_ans ;//V_C是速度常数，可调 pid结果改为0.5
                        mode4_v_L=2*limit_control(mode4_v_L, -20, 20);
                        mode4_v_R=2*limit_control(mode4_v_R, -20, 20); //限幅可改
                        Load(mode4_v_L, mode4_v_R, mode4_v_L, mode4_v_R);
                        if (mode4_huidu_data_sum!=8) {//不等于8，识别到黑色
                                    brake();
                                    Sign_LED_Bee();
                                    mode4_flag=4;
                        }

                    }
                    else if (mode4_flag==4) {
                        mode4_huidu_updata();//更新灰度

                        if (mode4_huidu_data[0]==0) {
                            speed_left = -2*speed_const;
                            speed_right = 4.5*speed_const;
                        }
                        else if (mode4_huidu_data[7]==0) {
                            speed_left = 4.5*speed_const;
                            speed_right  = -3*speed_const;
                        }
                        if (mode4_huidu_data[1]==0) {
                            speed_left = 0.8*speed_const;
                            speed_right = 4.1*speed_const;
                        }
                        else if (mode4_huidu_data[6]==0) {
                            speed_left = 4.1*speed_const;
                            speed_right  = 0.8*speed_const ;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==1) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 3.1*speed_const;
                        }

                        else if (mode4_huidu_data[2]==0&&mode4_huidu_data[3]==0) {
                            speed_left = 0.5*speed_const ;
                            speed_right = 2.6*speed_const;
                        }

                        else if (mode4_huidu_data[3]==0&&mode4_huidu_data[4]==0) {
                            speed_left = 2*speed_const;
                            speed_right = 0.8*speed_const;
                        }
                        else if (mode4_huidu_data[4]==0&&mode4_huidu_data[5]==0) {
                            speed_left = 2.6*speed_const ;
                            speed_right = 0.5*speed_const;
                        }

                        else if (mode4_huidu_data[4]==1 && mode4_huidu_data[5]==0) {
                            // speed_left = 0.8*speed_const;
                            //speed_left+=3;
                            //speed_right -=3;
                            speed_left = 3.1*speed_const ;
                            speed_right = 0.5*speed_const;
                        }
                        if ( mode4_huidu_data_sum==8 ) {//退出巡线条件
                            brake();
                            Sign_LED_Bee();
                            mode4_flag=5;
                        }
                        else {
                            speed_left = limit_control(speed_left, -30, 70);
                            speed_right = limit_control(speed_right, -30, 70);
                            Load(speed_left, speed_right, speed_left, speed_right);
                        }
                    }
                }
                else if (mode == 0) {
                    // Load(speed_left, speed_right, speed_left, speed_right);
                    // encoder_read(&EncoderA,&EncoderB, &EncoderC,&EncoderD);//更新编码器值
                    begin=0;
                    brake();
                    huidu_updata();
                    mode4_huidu_updata();
                }
                else {
                    brake();
                    begin=0;//mode 值都不对，则返回begin 0 
                }

            }
        }
        else 
        {
            huidu_updata();
            mode4_huidu_updata();
            if(mode>7) {
                brake();
                mode=0;
            }
        }
			break;

			default:
				break;
			}
}


int Mode4_line_PID(float reality_angle, float target_angle)
{
    //预计采用PD算法，不用积分项
    static float Error_Angle, Last_Error, Integral_error, PWM_line=0;

    Error_Angle = reality_angle-target_angle;//计算偏差
    
    // if (fabsf(Error_Angle)<=0.5) {
    //     return 0;
    // }
    // Integral_error+=Error_Angle;//累计偏差
    // else {
        PWM_line = (Mode4_line_Kp*Error_Angle)    //比例环节
                + (Mode4_line_Kp*(Error_Angle-Last_Error)); //微分环节

        Last_Error=Error_Angle;
        return 10*PWM_line;//结果乘10倍再保存
    // }
}


// int Position_PID(int reality,int target)//走直线的PID函数，勿动
// { 	
//     static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
//     Bias=reality-target;                            /* 计算偏差 */
//     Integral_bias+=Bias;	                        /* 偏差累积 */
    
//     if(Integral_bias> Integral_bias_MAX) Integral_bias = Integral_bias_MAX;   /* 积分限幅 */
//     if(Integral_bias<-Integral_bias_MAX) Integral_bias = -Integral_bias_MIN;
    
//     Pwm = (Position_KP*Bias)                        /* 比例环节 */
//          +(Position_KI*Integral_bias)               /* 积分环节 */
//          +(Position_KD*(Bias-Last_Bias));           /* 微分环节 */
    
//     Last_Bias=Bias;                                 /* 保存上次偏差 */
//     return Pwm;                                     /* 输出结果 */
// }

// 独立灰度更新函数
void mode4_huidu_updata()
{
    mode4_huidu_read_status = DL_GPIO_readPins( GPIOA,
                GPIO_Sensor_PIN_huidu6_PIN | GPIO_Sensor_PIN_huidu5_PIN | GPIO_Sensor_PIN_huidu4_PIN |
                GPIO_Sensor_PIN_huidu3_PIN| GPIO_Sensor_PIN_huidu2_PIN | GPIO_Sensor_PIN_huidu1_PIN |
                GPIO_Sensor_PIN_huidu0_PIN| GPIO_Sensor_PIN_huidu7_PIN);
    mode4_huidu_data_sum=0;//求和前清零
    for (i=0;i<=7;i++) {
        mode4_huidu_data_sum+=mode4_huidu_data[i];//更新求和
    }
    if ((mode4_huidu_read_status & GPIO_Sensor_PIN_huidu1_PIN)) {
    mode4_huidu_data[1]=1;
    }
    else {
    mode4_huidu_data[1]=0;
    }

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu2_PIN) {
        mode4_huidu_data[2]=1;
    }
    else {
        mode4_huidu_data[2]=0;
    }

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu3_PIN) {
        mode4_huidu_data[3]=1;
    }
    else {
        mode4_huidu_data[3]=0;
    }

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu4_PIN) {
        mode4_huidu_data[4]=1;
        }
    else {
    mode4_huidu_data[4]=0;
    }

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu5_PIN) {
    mode4_huidu_data[5]=1;
    }
    else {
    mode4_huidu_data[5]=0;
    }

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu6_PIN) {
    mode4_huidu_data[6]=1;
    }
    else {
    mode4_huidu_data[6]=0;
    }
    //----------------------------------------------------------

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu0_PIN) {
    mode4_huidu_data[0]=1;
    }
    else {
    mode4_huidu_data[0]=0;
    }

    if (mode4_huidu_read_status & GPIO_Sensor_PIN_huidu7_PIN) {
    mode4_huidu_data[7]=1;
    }
    else {
    mode4_huidu_data[7]=0;
    }

}