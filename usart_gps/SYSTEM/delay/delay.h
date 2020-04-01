#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  

////////////////////////////////////////////////////////////////////////////////// 

//滴答定时器初始化
void delay_init(void);
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(u16 nms);


//延时nus
//nus为要延时的us数.	 参数不得超过1864000
void delay_us(u32 nus);

//开启滴答定时器计时及中断   参数不得超过1864
void time_sys_start(u16 nms);

//关闭滴答定时器计时及中断
void time_sys_stop(void);


//查询滴答定时器当前倒计时值
u32 poll_time_sys_ms(void);

#endif





























