/**
  ******************************************************************************
  * @file    Examples_LL/TIM/TIM_PWMOutput/Src/stm32f4xx_timer.c
  * @author  MCD Application Team
  * @brief   This example describes how to use a timer peripheral to generate a 
  *          PWM output signal and update PWM duty cycle
  *          using the STM32F4xx TIM LL API.
  *          Peripheral initialization done using LL unitary services functions.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_LL_Examples
  * @{
  */

/** @addtogroup TIM_PWMOutput
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Number of output compare modes */
#define TIM_DUTY_CYCLES_NB 11

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Duty cycles: D = T/P * 100%                                                */
/* where T is the pulse duration and P  the period of the PWM signal          */
static uint32_t aDutyCycle[TIM_DUTY_CYCLES_NB] = {
  0,    /*  0% */
  10,   /* 10% */
  20,   /* 20% */
  30,   /* 30% */
  40,   /* 40% */
  50,   /* 50% */
  60,   /* 60% */
  70,   /* 70% */
  80,   /* 80% */
  90,   /* 90% */
  100,  /* 100% */
};


uint32_t TIM1ClockEN_Flag = 1;
uint32_t CaptureDistValue = 0;
/* Measured duty cycle */
__IO uint32_t uwMeasuredDutyCycle = 0;

/* TIM2 Clock */
static uint32_t TimOutClock = 1;

/* TIM1 Clock */
static uint32_t InitialAutoreload = 0;
uint32_t system_tick = 0;
/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/




/**
  * @brief  Get system tick
  * @note   This Function is used to de get system tick value
  * @param  None
  * @retval system_tick
  */

uint32_t Get_Systick(void)
{
	return system_tick;
}


/**
  * @brief  SysTick update callback function
  * @note   This Function update system tick
  * @param  None
  * @retval None
  */

void SysTickUpdate_Callback(void)
{
	system_tick++;
}


/**
  * @brief  LL delay_us
  * @note   This Function is used to delay, uint is 1us
  * @param  None
  * @retval system_tick
  */

void LL_usDelay(uint32_t delay_us)
{

	static uint32_t start_tick = 0;
	static uint32_t end_tick = 0;
	static uint32_t CompareDiffTick = 0;
	
	start_tick = Get_Systick();

	while(CompareDiffTick < delay_us){
		end_tick = Get_Systick();
		if(end_tick < start_tick){
			CompareDiffTick = ((MAX_TIME_BASE - start_tick) + end_tick) + 1;
		}
		else{
			CompareDiffTick = end_tick - start_tick;
		}
		//dbg_printf("CompareDiff:%d\n",CompareDiffTick);
	}
	CompareDiffTick = 0;
}

/**
  * @brief  Timer capture/compare interrupt processing
  * @note TIM3 input capture module is used to capture the value of the counter
  *       after a transition is detected by the corresponding input channel.
  * @param  None
  * @retval None
  */
void Timer3Capture_Callback(void)
{
  /* Capture index */
  static uint16_t uhCaptureIndex = 0;
  
  /* Captured Values */
  static uint32_t uwICValue1 = 0;
  static uint32_t uwICValue2 = 0;
  static uint32_t uwDiffCapture = 0;
  
  static uint16_t cnt = 0;
	
  if(uhCaptureIndex == 0)
  {
    /* Get the 1st Input Capture value */
    uwICValue1 = Get_Systick();		
    uhCaptureIndex = 1;
  }
  else if(uhCaptureIndex == 1)
  {
    /* Get the 2nd Input Capture value */
    uwICValue2 = Get_Systick(); 
    
    /* Capture computation */
		
		if(uwICValue2 != uwICValue1){
			if (uwICValue2 > uwICValue1)
				uwDiffCapture = (uwICValue2 - uwICValue1);
			else
				uwDiffCapture = ((MAX_TIME_BASE - uwICValue1) + uwICValue2) + 1; 

			if(0 == (cnt++ % 10)){
				CaptureDistValue = 170 * uwDiffCapture / 1000;
				dbg_printf("Capture Vaule:%dmm\n",CaptureDistValue);
			}
			
		}
		else{
			dbg_printf("Get Raise and Fall time ErrorStatus\n");
		}
		
    /* reset capture index */
    uhCaptureIndex = 0;    
  }
}


/**
  * @brief  Configures the timer to generate 10ms tick.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void  Configure_TIM1TimeBase(void)
{
  /* Enable the timer peripheral clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1); 
  
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIM1 counter clock equal to 10 kHz      */
  /*
    In this example TIM2 input clock (TIM2CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM2CLK = PCLK1
      PCLK1 = HCLK
      => TIM2CLK = HCLK = SystemCoreClock
    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
  */
  LL_TIM_SetPrescaler(TIM1, __LL_TIM_CALC_PSC(SystemCoreClock, INIT_TIM1_PSC_V));
  
  /* Set the auto-reload value to have an initial update event frequency of 10 Hz */
    /* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
  TimOutClock = SystemCoreClock;
  
  InitialAutoreload = __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM1), 20);
  LL_TIM_SetAutoReload(TIM1, InitialAutoreload);
  
  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM1);
  
  /* Configure the NVIC to handle TIM1 update interrupt */
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM1);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM1);
}


/**
  * @brief  This function enables the peripheral clock on TIM3, configures
  *         TIM3_CH1 as input and enables the capture/compare 1 interrupt
  *         It enables also the peripheral clock for GPIOA and configures 
  *         PA.06 as alternate function for TIM3_CH1.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_TIMInputCapture(void)
{
  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* GPIO TIM3_CH4 configuration */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_2);

  /***************************************************************/
  /* Configure the NVIC to handle TIM3 capture/compare interrupt */
  /***************************************************************/
  NVIC_SetPriority(TIM3_IRQn, 2);
  NVIC_EnableIRQ(TIM3_IRQn);
  
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  
  /************************************/
  /* Input capture mode configuration */
  /************************************/
  /* Select the active input: IC1 = TI1FP1 */
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  
  /* Configure the input filter duration: no filter needed */
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);

  /* Set input prescaler: prescaler is disabled */
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);

  /* Select the edge of the active transition on the TI4 channel: both rising and falling edge */
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_BOTHEDGE);
  
  /**************************/
  /* TIM3 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1 */
  LL_TIM_EnableIT_CC1(TIM3);
  
  /***********************/
  /* Start input capture */
  /***********************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    
  /* Enable counter */
  LL_TIM_EnableCounter(TIM3);
}


/**
  * @brief  Configures the timer to generate a PWM signal on the OC1 output.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void  Configure_TIMPWMOutput(void)
{
  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  /* GPIO TIM2_CH1 configuration */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_1);
  
  /***********************************************/
  /* Configure the NVIC to handle TIM2 interrupt */
  /***********************************************/
  NVIC_SetPriority(TIM2_IRQn, 2);
  NVIC_EnableIRQ(TIM2_IRQn);
  
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); 
  
  /***************************/
  /* Time base configuration */
  /***************************/
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  //LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  
  /* Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz */
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));
  
  /* Enable TIM2_ARR register preload. Writing to or reading from the         */
  /* auto-reload register accesses the preload register. The content of the   */
  /* preload register are transferred into the shadow register at each update */
  /* event (UEV).                                                             */  
  LL_TIM_EnableARRPreload(TIM2);
  
  /* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  TimOutClock = SystemCoreClock/1;
  LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM2), 100));
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output mode */
  /* Reset value is LL_TIM_OCMODE_FROZEN */
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  
  /* Set output channel polarity */
  /* Reset value is LL_TIM_OCPOLARITY_HIGH */
  //LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  
  /* Set compare value to half of the counter period (50% duty cycle ) */
  LL_TIM_OC_SetCompareCH1(TIM2, ( (LL_TIM_GetAutoReload(TIM2) + 1 ) / 4));
  
  /* Enable TIM2_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIM2_CCR1 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  
  /**************************/
  /* TIM2 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1*/
  LL_TIM_EnableIT_CC1(TIM2);
  
  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM2);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM2);
}

/**
  * @brief  Changes the duty cycle of the PWM signal.
  *         D = (T/P)*100
  *           where T is the pulse duration and P is the PWM signal period
  * @param  D Duty cycle
  * @retval None
  */
#if 0
__STATIC_INLINE
#endif
void Configure_DutyCycle(uint32_t D)
{
  uint32_t P;    /* Pulse duration */
  uint32_t T;    /* PWM signal period */
  
  /* PWM signal period is determined by the value of the auto-reload register */
  T = LL_TIM_GetAutoReload(TIM2) + 1;
  
  /* Pulse duration is determined by the value of the compare register.       */
  /* Its value is calculated in order to match the requested duty cycle.      */
  P = (D*T)/100;
  LL_TIM_OC_SetCompareCH1(TIM2, P);
}


/**
  * @brief  Timer1 capture/compare interrupt processing
  * @param  None
  * @retval None
  */
void Timer1UpdateCounter_Callback(void)
{
	TIM1ClockEN_Flag = 1;
}



/**
  * @brief  Timer capture/compare interrupt processing
  * @param  None
  * @retval None
  */
void TimerCaptureCompare_Callback(void)
{
	static uint32_t callback_cnt = 0;
  uwMeasuredDutyCycle = (LL_TIM_GetCounter(TIM2) * 100) / ( LL_TIM_GetAutoReload(TIM2) + 1 );
	
	if((callback_cnt & 0xffff) == 0xffff){
		dbg_printf("callback duty_cycle %d %\n", uwMeasuredDutyCycle);
	}
	callback_cnt = (callback_cnt + 1) & 0xff;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
