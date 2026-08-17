//#include "ti_msp_dl_config.h"
#include "motor.h"
#include <limits.h>
#include <ti/driverlib/dl_timerg.h>

int PWM_MAX =90, PWM_MIN = -90;

void Limit(int *motorA, int *motorB, int *motorC, int *motorD)
{
	if(*motorA > PWM_MAX)
	{
		*motorA = PWM_MAX;
	}
	if(*motorA < PWM_MIN)
	{
		*motorA = PWM_MIN;
	}
	if(*motorB > PWM_MAX)
	{
		*motorB = PWM_MAX;
	}
	if(*motorB < PWM_MIN)
	{
		*motorB = PWM_MIN;
	}
    ////
    if(*motorC > PWM_MAX)
	{
		*motorC = PWM_MAX;
	}
	if(*motorC < PWM_MIN)
	{
		*motorC = PWM_MIN;
	}
    if(*motorD > PWM_MAX)
	{
		*motorD = PWM_MAX;
	}
	if(*motorD < PWM_MIN)
	{
		*motorD = PWM_MIN;
	}
    
}

int my_abs(int p)
{
    if (p == INT_MIN) {
        return INT_MAX;
    }
    return p < 0 ? -p : p;
}

int get_abs(int x)
{
    return my_abs(x);
}

void Load(int motor1, int motor2 , int motor3 , int motor4)
{
    if(motor1 > 0)
    {
        AIN1_OUT(1);
        AIN2_OUT(0);
    }
    else if(motor1 < 0)
    {
        motor1 = -motor1;
		
        AIN1_OUT(0);
        AIN2_OUT(1);
    }
    if(motor2 > 0)
    {
        BIN1_OUT(1);
        BIN2_OUT(0);
    }
    else if(motor2 < 0)
    {
        motor2 = -motor2;
        BIN1_OUT(0);
        BIN2_OUT(1);
    }
//
    if(motor3 > 0)
    {
        CIN1_OUT(0);
        CIN2_OUT(1);
    }
    else if (motor3<0) {
        motor3=-motor3;
        CIN1_OUT(1);
        CIN2_OUT(0);
    }
//
    if (motor4 > 0) {
        DIN1_OUT(1);
        DIN2_OUT(0);
    }
    else if (motor4<0) {
        motor4 = -motor4;
        DIN1_OUT(0);
        DIN2_OUT(1);
    }

DL_TimerG_setCaptureCompareValue(TIMA0,motor1,DL_TIMER_CC_0_INDEX);
DL_TimerG_setCaptureCompareValue(TIMA0,motor2,DL_TIMER_CC_1_INDEX); //
DL_TimerG_setCaptureCompareValue(TIMA0,motor3,DL_TIMER_CC_2_INDEX);
DL_TimerG_setCaptureCompareValue(TIMA0,motor4,DL_TIMER_CC_3_INDEX);

}

//刹车函数
void brake(void)
{
        DL_TimerG_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_0_INDEX);
        DL_TimerG_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_1_INDEX);
        DL_TimerG_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_2_INDEX);
        DL_TimerG_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_3_INDEX);

        AIN1_OUT(1);
        AIN2_OUT(1);

	    BIN1_OUT(1);
        BIN2_OUT(1);

        CIN1_OUT(1);
        CIN2_OUT(1);

        DIN1_OUT(1);
        DIN2_OUT(1);

}