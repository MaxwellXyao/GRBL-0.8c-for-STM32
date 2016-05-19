【SOL开源】GRBL_0.8c_for_stm32

移植说明：

移植平台：STM32F103C8T6

目前进度：
（1）已经调试通过，无warning；
（2）定时器，串口，eeprom运行正常；
（3）在Grbl Controller下可以正常跑完G代码；

待改进：
（1）stepper.c部分代码直接移植，还没有针对stm32进行完全优化；
（2）limit还有待研究；
（3）还没有实机试验过；


/*--------------------------------
引脚分布情况：
***********************************
已用：
步进电机组：	GPIOB 5-11	   	推挽OUT
冷却：		GPIOB 0,1 		推挽OUT
主轴控制：	GPIOA 11,12		推挽OUT
限位开关：	GPIOA 6,7,8		IN	（共用中断5-9）
其他引脚：	GPIOA 0,1,2		IN	（独立中断0,1,2）		
USART1：	GPIOA 9,10		定义在HW_USART.c中

【由于涉及到中断，最好不要改动】

***********************************
剩余：
空闲：		GPIOA 3,4,5 		可加OLED
SPI2:		GPIOB 12-15		可加SD卡
调试：		GPIOA 13,14,15		可改为普通IO
调试&BOOT0	GPIOB 2,3,4		

--------------------------------*/


2015-12-31
SOL.lab
MaxwellXyao
email:MaxwellXyao@foxmail.com
