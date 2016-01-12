#include "include.h"

//####################################【初始化】####################################

void HW_USART_Init(u32 bound)
{  	 
	USART_InitTypeDef USART_InitStructure;
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F; 
	GPIOA->CRH|=0X000008B0;//IO状态设置

	USART_InitStructure.USART_BaudRate = bound;//波特率设置;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为 8 位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx |USART_Mode_Tx;//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断
	MY_NVIC_Init(3,3,USART1_IRQChannel,2);//最低优先级		

	USART_Cmd(USART1, ENABLE); //使能串口
}


//####################################【中断】####################################


/*-------------------------------------------------------------------
USART1的接收中断函数USART1_IRQHandler()从stm32f10x_it.c中移到该文件
中暂存，将要移植到serial.c中
然后这里要注释掉!!!!!

-------------------------------------------------------------------*/
//-------------------[接收中断函数]


//void USART1_IRQHandler(void)
//{
//	if(USART_GetFlagStatus(USART1 , USART_IT_RXNE)!=RESET)//接收到数据
//	{
//		//-----------------【接收】
////		rx_data=USART1->DR;				   //接收完自动清中断
//
//		//--------------------------
//	}
//	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) 	//写数据寄存器空，可以写数据
//	{
//		//-----------------【发送】
////		USART1->DR=tx_data;
//
//		//--------------------------
//		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//TXE中断必须手动关，否则只要空了，就会进中断
//	}
//}




