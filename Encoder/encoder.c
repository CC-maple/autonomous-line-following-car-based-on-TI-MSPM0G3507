#include "encoder.h"
#include "ti_msp_dl_config.h"//用户自定义名称所在头文件
#include "control.h"

int cnt = 0;
int64_t enc_cnt[4];
// uint8_t mode=4;//
uint8_t mode=0;
uint8_t begin=0;

void Encoder_Init(void)
{
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void GROUP1_IRQHandler(void)
{
     uint32_t gpio_Switch = DL_GPIO_getEnabledInterruptStatus(GPIO_Switch_PORT, 
     GPIO_Switch_PIN_S1_PIN | GPIO_Switch_PIN_S2_PIN | GPIO_Switch_PIN_S3_PIN | GPIO_Switch_PIN_S4_PIN);
     
     if (gpio_Switch&GPIO_Switch_PIN_S1_PIN) {
        begin = 1;
        DL_GPIO_clearInterruptStatus(GPIO_Switch_PORT, GPIO_Switch_PIN_S1_PIN);
     }
     else if (gpio_Switch&GPIO_Switch_PIN_S2_PIN) {
        mode += 1;
        DL_GPIO_clearInterruptStatus(GPIO_Switch_PORT, GPIO_Switch_PIN_S2_PIN);
     }
     else if (gpio_Switch&GPIO_Switch_PIN_S3_PIN) {
        mode4_angle_change+=0.2;
        DL_GPIO_clearInterruptStatus(GPIO_Switch_PORT, GPIO_Switch_PIN_S3_PIN);
     }
     else if (gpio_Switch&GPIO_Switch_PIN_S4_PIN) {
        mode4_angle_change-=0.2;
        DL_GPIO_clearInterruptStatus(GPIO_Switch_PORT, GPIO_Switch_PIN_S4_PIN);
     }

     uint32_t status,flag;
		status = DL_GPIO_getEnabledInterruptStatus(GPIOB,DL_GPIO_PIN_4 | DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 |
        DL_GPIO_PIN_22|DL_GPIO_PIN_5 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11);
	  DL_GPIO_clearInterruptStatus(GPIOB,status);

    if((status & DL_GPIO_PIN_7) == DL_GPIO_PIN_7) 
			
	{
		if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_8) == 0)
		{
			 enc_cnt[0] --;
		}
		// DL_GPIO_clearInterruptStatus(GPIOB,DL_GPIO_PIN_7);
	}
	else if((status & DL_GPIO_PIN_8) == DL_GPIO_PIN_8)
	{
		if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_7) == 0)
		{
			enc_cnt[0] ++;
		}
		// DL_GPIO_clearInterruptStatus(GPIOB,DL_GPIO_PIN_8);
	}
	
	if((status & DL_GPIO_PIN_4) == DL_GPIO_PIN_4)
	{
		if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_6) == 0)
		{
			enc_cnt[1] --;
		}
		// DL_GPIO_clearInterruptStatus(GPIOB,DL_GPIO_PIN_4);
	}
	else if((status & DL_GPIO_PIN_6) == DL_GPIO_PIN_6)
	{
		if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_4) == 0)
		{
			enc_cnt[1] ++;
		}
		// DL_GPIO_clearInterruptStatus(GPIOB,DL_GPIO_PIN_6);
	}

    if ((status & DL_GPIO_PIN_22) == DL_GPIO_PIN_22) {
        if ( DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_5) == 0) {
            enc_cnt[2]++;
        }
    }
    else if ((status & DL_GPIO_PIN_5)==DL_GPIO_PIN_5) {
        if ( DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_22) == 0) {
            enc_cnt[2]--;
        }
    }

    if ( (status & DL_GPIO_PIN_10) == DL_GPIO_PIN_10 ) {
        if ( DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_11) == 0) {
            enc_cnt[3]++;
        }
    }

    else if ((status & DL_GPIO_PIN_11) == DL_GPIO_PIN_11) {
        if ( DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_10) == 0) {
            enc_cnt[3]--;
        }
    }
    
}


void encoder_read(int64_t *a,int64_t *b, int64_t *c, int64_t *d)
{	
	//暂存
	*a = -enc_cnt[0];
	*b =  enc_cnt[1];
    *c = enc_cnt[2];
    *d = enc_cnt[3];
	
	//清零
	enc_cnt[0] = 0;
	enc_cnt[1] = 0;
    enc_cnt[2] = 0;
    enc_cnt[3] = 0;
}


