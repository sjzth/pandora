#ifndef _IO_H
#define _IO_H
#include "sys.h"

/*********************************************************************************
			  ___   _     _____  _____  _   _  _____  _____  _   __
			 / _ \ | |   |_   _||  ___|| \ | ||_   _||  ___|| | / /
			/ /_\ \| |     | |  | |__  |  \| |  | |  | |__  | |/ /
			|  _  || |     | |  |  __| | . ` |  | |  |  __| |    \
			| | | || |_____| |_ | |___ | |\  |  | |  | |___ | |\  \
			\_| |_/\_____/\___/ \____/ \_| \_/  \_/  \____/ \_| \_/

 *	******************************************************************************
 *	本程序只供学习使用，未经作者许可，不得用于其它任何用途
 *	ALIENTEK Pandora STM32L475 IOT开发板
 *	LED驱动代码
 *	正点原子@ALIENTEK
 *	技术论坛:www.openedv.com
 *	创建日期:2018/10/27
 *	版本：V1.0
 *	版权所有，盗版必究。
 *	Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 *	All rights reserved
 *	******************************************************************************
 *	初始版本
 *	******************************************************************************/
 
#define keyup 0
#define key0  1
#define key1  2
#define key2  3
#define keyno 4

//RGB接口定义
#define LED_R(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET))
#define LED_R_TogglePin		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7)	//LED_R电平翻转

#define LED_G(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET))
#define LED_G_TogglePin     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8)	//LED_G电平翻转

#define LED_B(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET))
#define LED_B_TogglePin     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9)	//LED_B电平翻转

//蜂鸣器接口定义
#define BEEP(n)			(n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET))
#define BEEP_TogglePin		 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2)	//BEEP电平翻转

//key接口定义
#define KEY_UP  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) 
#define KEY_0   HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10) 
#define KEY_1   HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9) 
#define KEY_2   HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8) 
void LED_Init(void);
u8 keyscan(u8 mode);
u8 KeyLedApp(void);
#endif




