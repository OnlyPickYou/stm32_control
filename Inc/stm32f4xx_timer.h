/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_Tx/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_TIMER_H
#define __STM32F4xx_TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Export Private variables */
extern uint32_t system_tick;
extern uint32_t TIM1ClockEN_Flag;

/* Exported macro ------------------------------------------------------------*/
#define INIT_TIM1_PSC_V					(10000 - 1)				// 10000 divison, system clock time / (INIT_TIM1_PSC_V+1) = 10KHz
#define MAX_TIME_BASE						(0xFFFFFFFF)


/* Exported functions ------------------------------------------------------- */
void    Configure_TIM1TimeBase(void);
void    Configure_TIMPWMOutput(void);
void    Configure_DutyCycle(uint32_t OCMode);
void 		Configure_TIMInputCapture(void);
void    SysTickUpdate_Callback(void);
void 		Timer3Capture_Callback(void);
void 		TimerCaptureCompare_Callback(void);
void		Timer1UpdateCounter_Callback(void);
void 		LL_usDelay(uint32_t delay_us);


uint32_t Get_Systick(void);
																																																																																				
#endif /* __STM32F4XX_TIMER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
