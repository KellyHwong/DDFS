/*
 *  DDFS.h
 *  Created on: 2015年5月1日
 *  Author: HuangKan
 *  Description: TODO
 */

#ifndef DDFS_H_
#define DDFS_H_
#include <stdint.h>
typedef struct
{
	uint32_t waveChoose;
	uint32_t freq;
	uint32_t pwmPeroid;
	uint32_t pwmNum;//波形表中的数据
	uint32_t pwmWidth;//实际设置的占空比
	uint32_t tablePointer;
	float ddfsDiff;
	float accPhase;
}DDFSAppState;
//常量 周期采样数
#define CYCLE_NUM 1000
//#define PWM_FREQ 20000
//#define MAX_DUTY 2000
#define PWM_FREQ 40000
#define MAX_DUTY 1200

void PWMIntHandler();
void PWM_init();
void APPinit();
void SWIntHandler();
void SW_init();
void APPStatusUpdate();





#endif /* DDFS_H_ */
