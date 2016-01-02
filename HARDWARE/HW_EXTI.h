#ifndef __HW_EXTI_H
#define __HW_EXTI_H

#define cli()      NVIC_SETPRIMASK()  			//总中断关闭
#define sei()      NVIC_RESETPRIMASK()			//总中断开启


//##########################【API】##########################
void HW_EXTI_Init(void);

#endif
