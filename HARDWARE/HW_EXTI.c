#include "include.h"

void HW_EXTI_Init(void)
{
//-------------------【Limit】
#ifndef LIMIT_SWITCHES_ACTIVE_HIGH					 //预拉高，触发要下降沿
	Ex_NVIC_Config(0,X_LIMIT_PIN,FTIR);		 		//GPIOA
	Ex_NVIC_Config(0,Y_LIMIT_PIN,FTIR);		 		//GPIOA
	Ex_NVIC_Config(0,Z_LIMIT_PIN,FTIR);		 		//GPIOA	
#else // LIMIT_SWITCHES_ACTIVE_HIGH					 //预拉低，触发要上降沿
	Ex_NVIC_Config(0,X_LIMIT_PIN,RTIR);		 		//GPIOA
	Ex_NVIC_Config(0,Y_LIMIT_PIN,RTIR);		 		//GPIOA
	Ex_NVIC_Config(0,Z_LIMIT_PIN,RTIR);		 		//GPIOA	
#endif // !LIMIT_SWITCHES_ACTIVE_HIGH			  
	MY_NVIC_Init(0,0,EXTI9_5_IRQChannel,2);			//抢占0,子优先级0，组2

//-------------------【Other_Pin】				//按键预上拉，需要下降沿触发
	Ex_NVIC_Config(0,OTHER_RESET_PIN,FTIR);		 	//GPIOA			  
	MY_NVIC_Init(0,0,EXTI0_IRQChannel,2);			//抢占0,子优先级0，组2

	Ex_NVIC_Config(0,OTHER_FEED_HOLD_PIN,FTIR);		//GPIOA			  
	MY_NVIC_Init(0,0,EXTI1_IRQChannel,2);			//抢占0,子优先级0，组2

	Ex_NVIC_Config(0,OTHER_CYCLE_START_PIN,FTIR);	//GPIOA			  
	MY_NVIC_Init(0,0,EXTI2_IRQChannel,2);			//抢占0,子优先级0，组2

}


//##########################【独立中断】##########################
/*---------------------------------------------------------------

该处的中断执行函数从stm32f10x_it.c中剪切过来，调试完毕后要移植到limit.c和
protocol.c中去，然后将此处的用到的函数注释掉。

stm32有5个独立的中断执行函数0~4，剩下5~9共用一个中断执行函数,10~15共用一
个中断函数。

---------------------------------------------------------------*/


//void EXTI0_IRQHandler(void)			  	//OTHER_RESET_PIN
//{
//	
//	EXTI->PR=1<<0;  //清除LINE0上的中断标志位
//}
//
//void EXTI1_IRQHandler(void)			  	//OTHER_FEED_HOLD_PIN
//{
//
//	EXTI->PR=1<<1;  //清除LINE1上的中断标志位 	
//}
//
//void EXTI2_IRQHandler(void)			  	//OTHER_CYCLE_START_PIN
//{
//
//	EXTI->PR=1<<2;  //清除LINE2上的中断标志位
//}

void EXTI3_IRQHandler(void)				//未使用
{
	
	EXTI->PR=1<<3;  //清除LINE3上的中断标志位
}

void EXTI4_IRQHandler(void)				//未使用
{
	
	EXTI->PR=1<<4;  //清除LINE4上的中断标志位
}

//void EXTI9_5_IRQHandler(void)		 	//要作为limit口共用中断
//{
//
//
//	EXTI->PR=1<<5;
//	EXTI->PR=1<<6;
//	EXTI->PR=1<<7;
//	EXTI->PR=1<<8;
//	EXTI->PR=1<<9;
//}

void EXTI15_10_IRQHandler(void)		  	//未使用
{

	EXTI->PR=1<<10;
	EXTI->PR=1<<11;
	EXTI->PR=1<<12;
	EXTI->PR=1<<13;
	EXTI->PR=1<<14;
	EXTI->PR=1<<15;
}



