#include "usart.h"	  


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 
 
//串口1相关变量	
u8 USART_RX_BUF[USART_REC_LEN];     //USART1接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  


//串口2相关变量	
u16 USART2_NUM=0;//USART缓存中总的有效数据数量
u16 start=0;    //USART2有效数据头下标
u16 end=0;      //USART2有效数据尾下标
u8 USART2_RX_BUF[USART_REC_LEN];     //USART2接收缓冲
u8 first_flag = 0;  //非数据接收导致end=start



void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
  //USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = bound;//串口波特率 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

//初始化串口2
void uart2_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	 
  //使能USART1，GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  //USART2_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2
   
  //USART2_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  

  //Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
  //USART 初始化设置
  USART_InitStructure.USART_BaudRate = bound;//串口波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE); 	
	
}


//读取串口2接收数据
/*
  参数：len--指定读取的字节数
        pbuf--数据缓存区指针
  返回值：读取的字节数量
*/
u16 usart2_read(u16 len,u8 *pbuf)
{
	  if(USART2_NUM ==0)  //无接收数据返回
			  return 0;
	  else{
        u16 ret=0;
        u16 i;
        u16 array_index; 
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//关闭串口接受中断
           
           ret = USART2_NUM < len ? USART2_NUM : len;
           for(i=0; i<ret; i++){
              array_index = (start+i)%USART_REC_LEN;
              *(pbuf+i) = USART2_RX_BUF[array_index];
           }
           if(len == USART2_NUM)
			     first_flag = 0;
           USART2_NUM -= ret;
           start = (start+ret) % USART_REC_LEN;				
				
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
        return ret;
	  }
}

//串口2发送数据
/*
  参数：send_data--要发送的数据
  返回值：无
*/
void usart2_send(u8 send_data){
	
	while((USART2->SR&0X40)==0); //等待上次数据发送结束
	USART2->DR = send_data;
			
}



//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误 
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0){//接收未完成
			
			if(USART_RX_STA&0x4000){//接收到了0x0d
				
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				
			}else {//还没收到0X0D
					
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else{
					
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}
				
			}
		}   		 
  } 

} 



void USART2_IRQHandler(void)                	//串口2中断服务程序_中断接收
{

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
	   if(first_flag > 0){
		  if(end == start){
			 start++;
			 if(start >= USART_REC_LEN)
			    start = 0;	
		  }					
	   }
		
     USART2_RX_BUF[end] =USART_ReceiveData(USART2);	//读取接收到的数据
	   end++;
			
     if(end >= USART_REC_LEN)
		 end = 0;
     if(++USART2_NUM > USART_REC_LEN)
        USART2_NUM = USART_REC_LEN;
  } 
} 


























