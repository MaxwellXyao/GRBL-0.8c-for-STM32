#include "include.h"

void HW_TIM_Init(void)		  
{
	RCC->APB1ENR|=(1<<1|1<<2);	//TIM3,4时钟使能(2:0,3:1,4:2)    
	  
	TIM3->DIER|=1<<0;   	//允许更新中断	  
//	TIM3->DIER|=1<<6;		//触发中断使能		???????????????????????????????????
  	MY_NVIC_Init(1,2,TIM3_IRQChannel,2);//抢占1，子优先级2，组2
	
	TIM4->DIER|=1<<0;   	//允许更新中断	  
//	TIM4->DIER|=1<<6;		//触发中断使能		???????????????????????????????????
  	MY_NVIC_Init(0,1,TIM4_IRQChannel,2);//抢占0，子优先级1，组2		 //TIM4要在TIM3运行中启动，必须比TIM3优先级高
	
	//这里只是设置定时器，设置定时值和启动在stepper.c中配置
		
}




//原来在stm32f10x_it.c中，暂时先存放在此，调好底层后需要复制到stepper.c中，并将此处的函数注释掉

//void TIM3_IRQHandler(void) 
//{
//	if(TIM3->SR&0X0001)//溢出中断
//	{
//		    				   				     	    	
//	}				   
//	TIM3->SR&=~(1<<0);//清除中断标志位 
//}
//
//void TIM4_IRQHandler(void)
//{
//	if(TIM4->SR&0X0001)//溢出中断
//	{
//				    				   				     	    	
//	}				   
//	TIM4->SR&=~(1<<0);//清除中断标志位 
//}











