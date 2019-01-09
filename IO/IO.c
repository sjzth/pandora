#include "io.h"
#include "delay.h"

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
//#define keyEn 0

/**
 * @brief	LED IO��ʼ������
 *
 * @param   void
 *
 * @return  void
 */
void LED_Init(void)
{
	/*
		LED-B	PE9
		LED-G	PE8
		LED-R	PE7	
	*/
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
}
#ifdef keyEn
/***********************key region start***********************/
u8 keyscan(u8 mode)
{
	static u8 relase = 0;
	if((KEY_UP == 0) && (KEY_0 == 1) && (KEY_1 == 1) && (KEY_2 == 1) && (relase == 0))//ȫ���ɿ���־
	{
		relase  = 1;	
	}
	if(((KEY_UP == 1) || (KEY_0 == 0) || (KEY_1 == 0) || (KEY_2 == 0)) && relase)//�ɿ������ⰴ������
	{
		relase  = 0;
		if(KEY_UP == 1) return keyup;
		if(KEY_0  == 0) return key0;
		if(KEY_1  == 0) return key1;
		if(KEY_2  == 0) return key2;
	}
	return keyno;
}

u8 KeyLedApp(void)
{
  u8 key;
  key = keyscan( 0 );
  switch(key)
  {
	 case keyup:
		 LED_R_TogglePin;			 
		 break;
	 case key0:
		 LED_G_TogglePin;
		 break;
	 case key1:
		 LED_B_TogglePin;
		 break;
	 case key2:
		 BEEP_TogglePin;
		 break;
	 default:
		 break;
	 return 1;
  }
  return 0;
}

#endif



