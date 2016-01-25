#ifndef __include_h
#define __include_h

//-----------------【GRBL可设置部分】
#include "config.h"	   				//设置项
#include "defaults.h"				//默认参数
#include "pin_map.h"				//引脚


//############################【STM32相关设置】##########################

#define F_CPU 72000000
#define M_PI  3.14159265358979323846

/******************************************************************
							【引脚定义】
******************************************************************/

/*--------------------------------
引脚分布情况：
***********************************
已用：
步进电机组：GPIOB 5-11	   	推挽OUT
冷却：		GPIOB 0,1 		推挽OUT
主轴控制：	GPIOA 11,12		推挽OUT
限位开关：	GPIOA 6,7,8		IN	（共用中断5-9）
其他引脚：	GPIOA 0,1,2		IN	（独立中断0,1,2）		
USART1：	GPIOA 9,10		定义在HW_USART.c中

由于涉及到中断，最好不要改动

***********************************
剩余：
空闲：		GPIOA 3,4,5 
SPI2:		GPIOB 12-15		
调试1：		GPIOA 13,14,15
调试2&BOOT0	GPIOB 2,3,4

--------------------------------*/

//----------------------------【步进电机】
#define STEP_GPIOx GPIOB

#define X_STEP_PIN 5	 		//x轴脉冲
#define Y_STEP_PIN 6	 		//y轴脉冲
#define Z_STEP_PIN 7	 		//z轴脉冲

#define X_DIRECTION_PIN	8 		//x轴方向
#define Y_DIRECTION_PIN	9 		//y轴方向
#define Z_DIRECTION_PIN	10 		//z轴方向

#define STEPPERS_DISABLE_PIN 11 //步进电机组失能

//输出口

//----------------------------【主轴】
#define SPINDLE_GPIOx GPIOA

#define SPINDLE_ENABLE_PIN 11	//主轴使能
#define SPINDLE_DIRECTION_PIN 12//主轴方向

//输出口

//----------------------------【冷却】
#define COOLANT_GPIOx GPIOB

#define COOLANT_FLOOD_PIN 0 	//气流冷却使能

#ifdef ENABLE_M7 
#define COOLANT_MIST_PIN 1 		//喷雾冷却使能
#endif

//输出口

//----------------------------【限位】
#define LIMIT_GPIOx GPIOA

#define X_LIMIT_PIN 6  		   	//x轴限位开关
#define Y_LIMIT_PIN 7		  	//y轴限位开关
#define Z_LIMIT_PIN 8 		  	//z轴限位开关

//输入口，要使用共用中断（EXTI9_5_IRQHandler()）
//改变上下拉配置要先设置其输出端口的值！！！！
	

//----------------------------【其他】
#define OTHER_GPIOx	GPIOA

#define OTHER_RESET_PIN 0		//软复位按键（可以保存当前的坐标）
#define OTHER_FEED_HOLD_PIN 1 	//暂停按键
#define OTHER_CYCLE_START_PIN 2 //继续按键

//输入口，可使用独立中断（0~2）
//改变上下拉配置要先设置其输出端口的值！！！！

/******************************************************************
							【EEPROM定义】
******************************************************************/


#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
#define STM32_FLASH_SIZE 	64 	 			//所选STM32的FLASH容量大小(单位为K)

#define STM32_EEPROM_ADDR_START	0x0800FC00	//作为EEPROM的起始地址（地址必须是偶数）
#define STM32_EEPROM_ADDR_MAX	0x400			//EEPROM容量(单位B)


//###############################【头文件】##############################
//-----------------【外部加入】
#include"stm32f10x_lib.h"
#include"system.h"
#include"delay.h"
#include"HW_GPIO.h"
#include"HW_EEPROM.h"
#include"HW_TIM.h"
#include"HW_USART.h"
#include"HW_EXTI.h"								   

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inttypes.h> 
#include <math.h>





//-----------------【GRBL内部使用】
#include "planner.h"				//Gcode运行协调
#include "nuts_bolts.h"				//【硬】GRBL共用函数
#include "stepper.h"				//【硬】步进电机驱动
#include "spindle_control.h"		//【硬】主轴电机开关驱动
#include "coolant_control.h"		//【硬】冷却开关驱动
#include "motion_control.h" 		//电机联动控制
#include "gcode.h"					//处理串口发送的Gcode部分
#include "protocol.h"				//【硬】处理串口发送的非Gcode部分
#include "limits.h"					//【硬】限位开关驱动
#include "report.h"					//人机交互界面
#include "settings.h"				//epprom参数的读写
#include "serial.h"					//【硬】串口驱动
#include "print.h"					//串口打印字符
#include "eeprom.h"				 	//【硬】epprom驱动




#endif
