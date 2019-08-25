/**
  ******************************************************************************
  * @file    irobot.h
  * @author  jszhang@artosyn.com
  * @brief   Header for irobot.c module
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IROBOT_H
#define __IROBOT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_robot_fsm.h"

/* Exported macro ------------------------------------------------------------*/

#define _HD_RET_SUCCESS_			(0)
#define _HD_RET_FAILED_				(-1)



/* Exported functions ------------------------------------------------------- */


void UltraSonic_TRIG1_On(void);
void UltraSonic_TRIG1_Off(void);

void UltraSonic_DetEn(void);

int User_init(void);
void Irobot_mainloop(void);
																																																																																				
#endif /* __IROBOT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
