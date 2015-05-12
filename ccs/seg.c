/*
 *  seg.c
 *  Created on: 2015年4月25日
 *  Author: HuangKan
 *  Description: seg管的API函数
 */
#include "seg.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
//共阳极数码管显示代码
uint8_t SEG_A_List[16]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,
						0x82,0xf8,0x80,0x90,0x88,0x83,
						0xc6,0xa1,0x86,0x8e};

 void seg_display_num(uint32_t dat)
{
	uint8_t i,j;
	uint32_t temp = 1;

	//使能端口D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//设置PD0、PD1、PD3为输出
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

	//显示4位十进制数
	for (i=0;i<4;i++)
	{
		//从低位到高位每次剥离出一位数字
		temp = 1;
		for (j=0;j<i;j++)
			temp *= 10;
		temp = (dat/temp)%10;

		//串行发送8为待显示数据
		for(j=0;j<8;j++)
		{
			//高位先发送
			if((SEG_A_List[temp]<<j)&0x80)
				GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 , 0x08);
			else
				GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 , 0x00);
			//移位时钟引脚给一上升沿，进行移动一位数据
			GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1, 0x01);
		}
	}
	//存储时钟引脚给一上升沿，进行并行驱动数码管
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1, 0x03);
}
