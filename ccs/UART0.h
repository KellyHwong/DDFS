/*
 *  UART0.h
 *  Created on: 2015年5月1日
 *  Author: HuangKan
 *  Description: TODO
 */

#ifndef UART0_H_
#define UART0_H_

void uart_send_string(char str[]);
void uart0_interrupt();
void uart0_init(void);

#endif /* UART0_H_ */
