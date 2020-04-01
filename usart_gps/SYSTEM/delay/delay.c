#include "delay.h"
////////////////////////////////////////////////////////////////////////////////// 	 

static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������
	
	


			   
//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()
{

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8  ��ϵͳʱ�ӵ�1/8
	fac_us=SystemCoreClock/8000000;				            //Ϊϵͳʱ�ӵ�1/8  
	fac_ms=(u16)fac_us*1000;					            //ÿ��ms��Ҫ��systickʱ����   
}								    


//��ʱnus
/*
  ������nus--��ʱ��us��
  ����ֵ����
*/								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
	SysTick->VAL=0x00;        					//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					//��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
/*
  ������nms--��ʱ��ms��
  ����ֵ����
*/	
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	  	    
} 
 
//�����δ�ʱ����ʱ���ж�
/*
  ������nms--��ʱ��ms��
  ����ֵ����
*/	
void time_sys_start(u16 nms)
{
	SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk ;	//��ʼ���� �������ж�

}

//�رյδ�ʱ����ʱ���ж�
void time_sys_stop(void){
		
	SysTick->CTRL&=~(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk);	//�رռ��������ж�
	SysTick->VAL =0X00;       					//��ռ�����	  
}


//��ѯ�δ�ʱ����ǰ����ʱֵ
/*
  ����ֵ����ǰ����ʱʵʱֵ  ��λ����
*/
u32 poll_time_sys_ms(void){
	
	u32 tmp= SysTick->VAL;
	
	return tmp/fac_ms;
	
}

//systick�жϷ�����,ʹ��ucosʱ�õ�
void SysTick_Handler(void)
{	
	SysTick->CTRL&=~(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk);	//�رռ�ʱ�����ж�
	SysTick->VAL =0X00;       					//��ռ�����	 
}







































