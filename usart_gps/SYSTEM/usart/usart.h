#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 




#define USART_REC_LEN  			2000  	//定义最大接收字节数 2000 

//串口1相关变量		  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //USART1接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	


//串口2相关变量	
extern u8  USART2_RX_BUF[USART_REC_LEN]; //USART2接收缓冲,最大USART_REC_LEN个字节
extern u16 USART2_NUM;//USART缓存中总的有效数据数量
extern u16 start;    //USART2有效数据头下标
extern u16 end;      //USART2有效数据尾下标
extern u8 first_flag;  //非数据接收导致end=start


//初始化串口1
void uart_init(u32 bound);

//初始化串口2
void uart2_init(u32 bound);

//读取串口2接收数据
u16 usart2_read(u16 len,u8 *pbuf);

//串口2发送数据
void usart2_send(u8 send_data);


#endif


