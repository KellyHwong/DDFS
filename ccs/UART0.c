/*
 *  UART0.c
 *  Created on: 2015年5月1日
 *  Author: HuangKan
 *  Description: TODO
 */
#include "UART0.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"

//向串口0发送一字符串
void uart_send_string(char str[])
{
	char length,i;
	length = strlen(str);

	//一个字符一个字符的打印到电脑上
	for (i=0;i<length;i++)
	{
		if (str[i] == '\n')
			UARTCharPut(UART0_BASE,'\r');
		UARTCharPut(UART0_BASE,str[i]);
	}
}

//串口0中断处理函数，该例中用来处理接收中断
void uart0_interrupt()
{
	 unsigned long ulStatus;
	 char lInChar;

	//获取串口0中断状态
    ulStatus = UARTIntStatus(UART0_BASE, true);
	//清除对应中断状态
    UARTIntClear(UART0_BASE, ulStatus);

	//接收接收缓冲区中全部数据，该例中FIFO关闭，数据缓冲区深度
	//为1B，所以缓冲区中只有一个字节的数据
	while(UARTCharsAvail(UART0_BASE))
    {
		//接收到数据后发回到电脑
        lInChar = UARTCharGetNonBlocking(UART0_BASE);
		UARTCharPut(UART0_BASE,lInChar);
    }
}

//串口0初始化函数
void uart0_init(void)
{
	//使能端口A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//使能外设串口0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//配置引脚功能
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

	//设置引脚模式
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//设置串口0时钟源，片内16M精准晶振
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//配置串口0，波特率，数据长度，停止位，校验位
	UARTConfigSetExpClk(UART0_BASE,16000000, 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	//关闭串口FIFO，使得接收缓冲区深度为1字节，这样当接收到一个字节的数据立马就能产生中断
	UARTFIFODisable(UART0_BASE);

	//串口0，中断处理函数注册
	UARTIntRegister(UART0_BASE,uart0_interrupt);
	//使能串口0接收中断
	UARTIntEnable(UART0_BASE,UART_INT_RX );
}

