#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "gps.h"

#define  USART2_TMP_LEN 150
u8 USART2_TMP[USART2_TMP_LEN];
 int main(void)
 {	
	u8 t;
	u8 len;	
 
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(115200);	 //串口1初始化为115200
	uart2_init(115200);  //串口2初始化为115200
	LED_Init();		  	   //初始化与LED连接的硬件接口 
  gps_config();        //gps初始化配置
	while(1)
	{
		if(USART_RX_STA&0x8000){		
			
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("\r\n您发送的消息为:\r\n");
			for(t=0;t<len;t++)
			{
				USART1->DR=USART_RX_BUF[t];
				while((USART1->SR&0X40)==0);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}else{
			
      LED1=!LED1;//闪烁LED,提示系统正在运行.
			delay_ms(100);
      receive(50);			
			
			printf("lat:%8.10f\r\n",follow_data.lat);
			printf("lon:%8.10f\r\n",follow_data.lon);
			printf("alt:%8.10f\r\n",follow_data.alt);
			printf("vy:%6.5f\r\n",follow_data.vy);
			printf("vx:%6.5f\r\n",follow_data.vx);
			printf("vz:%6.5f\r\n",follow_data.vz);
			printf("numSV:%d\r\n",follow_data.numSV);
			printf("mag_dec:%d\r\n",follow_data.mag_dec);
			printf("数据总长：%d\r\n",sizeof(EXYF_FOLLOW));
			
		}
	}	 
}


