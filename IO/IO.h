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
 *	������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 *	ALIENTEK Pandora STM32L475 IOT������
 *	LED��������
 *	����ԭ��@ALIENTEK
 *	������̳:www.openedv.com
 *	��������:2018/10/27
 *	�汾��V1.0
 *	��Ȩ���У�����ؾ���
 *	Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 *	All rights reserved
 *	******************************************************************************
 *	��ʼ�汾
 *	******************************************************************************/
 
#define keyup 0
#define key0  1
#define key1  2
#define key2  3
#define keyno 4

//RGB�ӿڶ���
#define LED_R(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET))
#define LED_R_TogglePin		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7)	//LED_R��ƽ��ת

#define LED_G(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET))
#define LED_G_TogglePin     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8)	//LED_G��ƽ��ת

#define LED_B(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET))
#define LED_B_TogglePin     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9)	//LED_B��ƽ��ת

//�������ӿڶ���
#define BEEP(n)			(n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET))
#define BEEP_TogglePin		 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2)	//BEEP��ƽ��ת

//key�ӿڶ���
#define KEY_UP  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) 
#define KEY_0   HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10) 
#define KEY_1   HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9) 
#define KEY_2   HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8) 
void LED_Init(void);
u8 keyscan(u8 mode);
u8 KeyLedApp(void);
#endif




