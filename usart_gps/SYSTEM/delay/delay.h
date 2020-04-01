#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  

////////////////////////////////////////////////////////////////////////////////// 

//�δ�ʱ����ʼ��
void delay_init(void);
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void delay_ms(u16 nms);


//��ʱnus
//nusΪҪ��ʱ��us��.	 �������ó���1864000
void delay_us(u32 nus);

//�����δ�ʱ����ʱ���ж�   �������ó���1864
void time_sys_start(u16 nms);

//�رյδ�ʱ����ʱ���ж�
void time_sys_stop(void);


//��ѯ�δ�ʱ����ǰ����ʱֵ
u32 poll_time_sys_ms(void);

#endif





























