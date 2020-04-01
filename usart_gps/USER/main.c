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
 
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(115200);	 //����1��ʼ��Ϊ115200
	uart2_init(115200);  //����2��ʼ��Ϊ115200
	LED_Init();		  	   //��ʼ����LED���ӵ�Ӳ���ӿ� 
  gps_config();        //gps��ʼ������
	while(1)
	{
		if(USART_RX_STA&0x8000){		
			
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			printf("\r\n�����͵���ϢΪ:\r\n");
			for(t=0;t<len;t++)
			{
				USART1->DR=USART_RX_BUF[t];
				while((USART1->SR&0X40)==0);//�ȴ����ͽ���
			}
			printf("\r\n\r\n");//���뻻��
			USART_RX_STA=0;
		}else{
			
      LED1=!LED1;//��˸LED,��ʾϵͳ��������.
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
			printf("�����ܳ���%d\r\n",sizeof(EXYF_FOLLOW));
			
		}
	}	 
}


