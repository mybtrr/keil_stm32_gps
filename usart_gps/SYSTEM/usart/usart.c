#include "usart.h"	  


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 
 
//����1��ر���	
u8 USART_RX_BUF[USART_REC_LEN];     //USART1���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  


//����2��ر���	
u16 USART2_NUM=0;//USART�������ܵ���Ч��������
u16 start=0;    //USART2��Ч����ͷ�±�
u16 end=0;      //USART2��Ч����β�±�
u8 USART2_RX_BUF[USART_REC_LEN];     //USART2���ջ���
u8 first_flag = 0;  //�����ݽ��յ���end=start



void uart_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
  //USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
  //USART1 ��ʼ������
  USART_InitStructure.USART_BaudRate = bound;//���ڲ����� 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

//��ʼ������2
void uart2_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	 
  //ʹ��USART1��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  //USART2_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2
   
  //USART2_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3  

  //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
  //USART ��ʼ������
  USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE); 	
	
}


//��ȡ����2��������
/*
  ������len--ָ����ȡ���ֽ���
        pbuf--���ݻ�����ָ��
  ����ֵ����ȡ���ֽ�����
*/
u16 usart2_read(u16 len,u8 *pbuf)
{
	  if(USART2_NUM ==0)  //�޽������ݷ���
			  return 0;
	  else{
        u16 ret=0;
        u16 i;
        u16 array_index; 
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//�رմ��ڽ����ж�
           
           ret = USART2_NUM < len ? USART2_NUM : len;
           for(i=0; i<ret; i++){
              array_index = (start+i)%USART_REC_LEN;
              *(pbuf+i) = USART2_RX_BUF[array_index];
           }
           if(len == USART2_NUM)
			     first_flag = 0;
           USART2_NUM -= ret;
           start = (start+ret) % USART_REC_LEN;				
				
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
        return ret;
	  }
}

//����2��������
/*
  ������send_data--Ҫ���͵�����
  ����ֵ����
*/
void usart2_send(u8 send_data){
	
	while((USART2->SR&0X40)==0); //�ȴ��ϴ����ݷ��ͽ���
	USART2->DR = send_data;
			
}



//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ��� 
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0){//����δ���
			
			if(USART_RX_STA&0x4000){//���յ���0x0d
				
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				
			}else {//��û�յ�0X0D
					
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else{
					
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}
				
			}
		}   		 
  } 

} 



void USART2_IRQHandler(void)                	//����2�жϷ������_�жϽ���
{

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
	   if(first_flag > 0){
		  if(end == start){
			 start++;
			 if(start >= USART_REC_LEN)
			    start = 0;	
		  }					
	   }
		
     USART2_RX_BUF[end] =USART_ReceiveData(USART2);	//��ȡ���յ�������
	   end++;
			
     if(end >= USART_REC_LEN)
		 end = 0;
     if(++USART2_NUM > USART_REC_LEN)
        USART2_NUM = USART_REC_LEN;
  } 
} 


























