#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 




#define USART_REC_LEN  			2000  	//�����������ֽ��� 2000 

//����1��ر���		  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //USART1���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	


//����2��ر���	
extern u8  USART2_RX_BUF[USART_REC_LEN]; //USART2���ջ���,���USART_REC_LEN���ֽ�
extern u16 USART2_NUM;//USART�������ܵ���Ч��������
extern u16 start;    //USART2��Ч����ͷ�±�
extern u16 end;      //USART2��Ч����β�±�
extern u8 first_flag;  //�����ݽ��յ���end=start


//��ʼ������1
void uart_init(u32 bound);

//��ʼ������2
void uart2_init(u32 bound);

//��ȡ����2��������
u16 usart2_read(u16 len,u8 *pbuf);

//����2��������
void usart2_send(u8 send_data);


#endif


