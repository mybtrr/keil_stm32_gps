#include "delay.h"
////////////////////////////////////////////////////////////////////////////////// 	 

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数
	
	


			   
//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8  即系统时钟的1/8
	fac_us=SystemCoreClock/8000000;				            //为系统时钟的1/8  
	fac_ms=(u16)fac_us*1000;					            //每个ms需要的systick时钟数   
}								    


//延时nus
/*
  参数：nus--延时的us数
  返回值：无
*/								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					//清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
/*
  参数：nms--延时的ms数
  返回值：无
*/	
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	  	    
} 
 
//开启滴答定时器计时及中断
/*
  参数：nms--延时的ms数
  返回值：无
*/	
void time_sys_start(u16 nms)
{
	SysTick->LOAD=(u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk ;	//开始倒数 ，开启中断

}

//关闭滴答定时器计时及中断
void time_sys_stop(void){
		
	SysTick->CTRL&=~(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk);	//关闭计数器及中断
	SysTick->VAL =0X00;       					//清空计数器	  
}


//查询滴答定时器当前倒计时值
/*
  返回值：当前倒计时实时值  单位毫秒
*/
u32 poll_time_sys_ms(void){
	
	u32 tmp= SysTick->VAL;
	
	return tmp/fac_ms;
	
}

//systick中断服务函数,使用ucos时用到
void SysTick_Handler(void)
{	
	SysTick->CTRL&=~(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk);	//关闭计时器及中断
	SysTick->VAL =0X00;       					//清空计数器	 
}







































