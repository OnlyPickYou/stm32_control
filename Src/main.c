/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_Tx/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to send bytes over USART IP using
  *          the STM32F4xx USART LL API.
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
#include "irobot.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>


/** @addtogroup STM32F4xx_LL_Examples
  * @{
  */

/** @addtogroup USART_Communication_Tx
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
    char uart_tx_buf[DBG_OUTPUT_BUFFER_SIZE];
	  char uart_rx_buf[DBG_INPUT_BUFFER_SIZE];
}dbg_log_type_t;

/* Private define ------------------------------------------------------------*/
uint8_t buffer[32];
uint8_t aTextInfoStart[] = "\r\nUSART Example : Enter characters to fill reception buffers.\r\n";

__IO uint32_t uwBufferReadyIndication = 0;
__IO uint32_t uwNbReceivedChars = 0;

dbg_log_type_t  dbg_log_type = {0};

#if (USE_TIMEOUT == 1)
#define USART_SEND_TIMEOUT_TXE_MS 10
#define USART_SEND_TIMEOUT_TC_MS  20
#endif /* USE_TIMEOUT */

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
#if (USE_TIMEOUT == 1)
uint32_t Timeout = 0; /* Variable used for Timeout management */
#endif /* USE_TIMEOUT */
__IO uint8_t ubButtonPress = 0;
uint8_t ubSend = 0;
const uint8_t aStringToSend[] = "STM32F4xx USART LL API Example : TX in Polling mode\r\nConfiguration UART 115200 bps, 8 data bit/1 stop bit/No parity/No HW flow control\r\n";


/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     Configure_USART(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
#if (USE_TIMEOUT == 1)
void     LED_Blinking(uint32_t Period);
#endif /* USE_TIMEOUT */
void     UserButton_Init(void);
void     WaitForUserButtonPress(void);
//void     BufferTransfer(void);
void 		 StartReception(void);
void     dbg_printf(const char *src_format,...);
void     USARTx_RxBufferParser(uint8_t *buffer);
void 		 LED_Blinking(uint32_t Period);

/* Private functions ---------------------------------------------------------*/


void LL_USART_SendString(USART_TypeDef *USARTx, char *str)
{
	char ch = 0;
	int len = strlen(str);
	for(int i=0; i<len; i++){
		ch = str[i];		
    /* Wait for TXE flag to be raised */
		while (!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));		
		LL_USART_TransmitData8(USARTx, ch);
		if(ch == '\n'){
			ch = '\r';
			while (!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));
			LL_USART_TransmitData8(USARTx, ch);
		}
	}
	/* If last char to be sent, clear TC flag */
	LL_USART_ClearFlag_TC(USARTx_INSTANCE); 
}

/*
* *function:used for printf debug information
* *param : __FUNCTION__ print function_name, __FILE__ print file name, __LINE__ print line
*/


void dbg_printf(const char *src_format,...)
{
	
	va_list args;
	va_start(args, src_format);
	
	vsnprintf(dbg_log_type.uart_tx_buf, DBG_OUTPUT_BUFFER_SIZE-1, src_format, args);
	
	LL_USART_SendString(USARTx_INSTANCE, dbg_log_type.uart_tx_buf);
	va_end(args);	
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the system clock to 100 MHz */
  SystemClock_Config();

  /* Initialize LED2 */
  LED_Init();

  /* Set LED2 Off */
  LED_Off();

  /* Initialize button in EXTI mode */
  UserButton_Init();

  /* Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();
	
  /* Initiate Continuous reception */
  StartReception();

	/* Wait for user push button press to start transfer */
  WaitForUserButtonPress();
	
	/* Initiate CH1 of TIM2 for pwm output */
	Configure_TIMPWMOutput();

	/* Initiate CH1 of TIM3 for Input capture */
	Configure_TIMInputCapture();
	
  User_init();

	/* Initiate TIM1 as base, Frequency is 10ms */
	Configure_TIM1TimeBase();
		
  /* Infinite loop */
  while (1)
  {

    USARTx_RxBufferParser((uint8_t *)(dbg_log_type.uart_rx_buf));
		Irobot_mainloop();
    /* transfer Tx buffer to PC application */
    //BufferTransfer();
		//LED_Blinking(LED_BLINK_SO_FAST);
	}
 
}

/**
  * @brief  This function configures USARTx Instance.
  * @note   This function is used to :
  *         -1- Enable GPIO clock and configures the USART pins.
  *         -2- Enable the USART peripheral clock and clock source.
  *         -3- Configure USART functional parameters.
  *         -4- Enable USART.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_USART(void)
{

  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  USARTx_GPIO_CLK_ENABLE();

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  USARTx_SET_TX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  USARTx_SET_RX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USARTx_IRQn, 3);  
  NVIC_EnableIRQ(USARTx_IRQn);	
	
  /* (3) Enable USART peripheral clock and clock source ***********************/
  USARTx_CLK_ENABLE();

  /* (4) Configure USART functional parameters ********************************/
  
  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USARTx_INSTANCE, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 100000000/APB_Div Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 100000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
  */
  LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, LL_USART_OVERSAMPLING_16, 115200); 

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USARTx_INSTANCE);
}


/**
  * @brief  This function prints user info on PC com port and initiates RX transfer
  * @param  None
  * @retval None
  */
void StartReception(void)
{
  /* Initializes Buffer swap mechanism :
     - 2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
     
  */
  uwNbReceivedChars = 0;
  uwBufferReadyIndication = 0;

  /* Print user info on PC com port */
  dbg_printf("%s",aTextInfoStart);

  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USARTx_INSTANCE);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
  LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
}

/**
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LED2_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  //LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  //LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
}

/**
  * @brief  Turn-on LED2.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

#if (USE_TIMEOUT == 1 || 1)
/**
  * @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Toggle IO in an infinite loop */
  while (1)
  {
		dbg_printf("_LINE:%d\n",__LINE__);
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
    LL_mDelay(Period);
  }
}
#endif /* USE_TIMEOUT */

/**
  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
  * @param  None 
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();

  /* Enable a rising trigger EXTI13 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 3);  
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn); 
}

/**
  * @brief  Wait for user push button press to start transfer.
  * @param  None 
  * @retval None
  */
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
  }
  ubSend = 0;
}

/**
  * @brief  Example of User callback in charge of consuming received data.
  * @param  None
  * @retval None
  */
void UserDataTreatment(uint8_t *DataBuffer, uint32_t Size)
{
  /* Display info message + buffer content on PC com port */
  dbg_printf("%s",__FUNCTION__);
	
  LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
}

/**
  * @brief  Function called for achieving TX buffer sending
  * @param  None
  * @retval None
  */


void USARTx_RxBufferParser(uint8_t *buffer)
{
		uint32_t start_tick = 0;
		uint32_t end_tick = 0;
	  uint32_t dutycycle = 0;
	  usart_cmd_ind_t *cmd_ind = NULL;
	  if(!uwBufferReadyIndication){
			return;
		}
		uwBufferReadyIndication = 0;
		
		if(buffer[0] == '\r'){
			dbg_printf("invalid command\n");
			dbg_printf("%s>>>","stm32");
			return;
		}
					
		if((buffer[0]==USART_RECP_HEAD_0)&&(buffer[1]==USART_RECP_HEAD_1)&&(buffer[2] == USART_RCEP_COMMAND)){
			dbg_printf("%s in\n",__FUNCTION__);
			cmd_ind = (usart_cmd_ind_t *)buffer;
			start_tick = Get_Systick();			
			switch(cmd_ind->idx){
				case COMMAND_TEST_ULTRASONIC:
					dbg_printf("ULTRASONIC TEST\n");
					
				  LED_On();
					break;
				case COMMAND_TEST_INFRARED:
					dbg_printf("INFRARED TEST\n");
					//LED_Blinking(LED_BLINK_SLOW);
				  LED_On();
					break;
				case COMMAND_TEST_E_MACHINE:
					dbg_printf("E-MACHINE TEST\n");
					LED_Off();
					break;
				case COMMAND_TEST_PWM_DUTY_CYCLE:
					dutycycle = (uint32_t)(cmd_ind->data[0] | (cmd_ind->data[1] << 8) | \
																	(cmd_ind->data[2] << 16) | (cmd_ind->data[3] << 24)); 
					dbg_printf("set %d-percent duty_cycle of pwm mode1 \n", dutycycle);
					Configure_DutyCycle(dutycycle);
					break;
				case COMMAND_TEST_NON_OBSTACLE:
					irobot_event_set(IROBOT_NON_OBSTACLE_EVENT);
					end_tick = Get_Systick();
					dbg_printf("start:%d,end:%d\n", start_tick, end_tick);
					break;
				case COMMAND_TEST_OBSTACLES:
					irobot_event_set(IROBOT_OBSTACLES_EVENT);
					end_tick = Get_Systick();
					dbg_printf("start:%d,end:%d\n", start_tick, end_tick);				
					break;				
				case COMMAND_TEST_CURRENT_IN:
					irobot_event_set(IROBOT_CURRENT_IN_EVENT);
					dbg_printf("start:%d, dur time:%d", start_tick, (Get_Systick() - start_tick));				
					break;				
				case COMMAND_TEST_CHARGE_FULL:
					irobot_event_set(IROBOT_CHARGE_FULL_EVENT);
					break;
				
				default:
					break;
			}
		}
		else{
			dbg_printf("command not match\n");
		}
				
}



#if 0
void BufferTransfer(void)
{
	
#if 1
  static unsigned int uart_send_cnt = 0;

#if (USE_TIMEOUT == 1)
  Timeout = USART_SEND_TIMEOUT_TXE_MS;
#endif /* USE_TIMEOUT */
	dbg_printf("%s %s %d, USART2 Tx Test,cnt %d\n",__FILE__, __FUNCTION__, __LINE__,uart_send_cnt++);

#if (USE_TIMEOUT == 1)
  Timeout = USART_SEND_TIMEOUT_TC_MS;
#endif /* USE_TIMEOUT */

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USARTx_INSTANCE))
  {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    { 
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    } 
#endif /* USE_TIMEOUT */
  }

  ubButtonPress =0;
	buffer[0] = uart_send_cnt;
	dbg_printf("buffer addr : %p, size %d, first param 0x%x\n", buffer, sizeof(buffer), buffer[0]);
    
  /* Turn LED2 On at end of transfer : Tx sequence completed successfully */
	if(uart_send_cnt & 1)
		LED_On();
	else
		LED_Off();
	
#else
	
  /* Send characters one per one, until last char to be sent */
  while (ubSend < sizeof(aStringToSend))
  {
#if (USE_TIMEOUT == 1)
    Timeout = USART_SEND_TIMEOUT_TXE_MS;
#endif /* USE_TIMEOUT */

    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE))
    {
#if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag()) 
      { 
        if(Timeout-- == 0)
        {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_SLOW);
        }
      } 
#endif /* USE_TIMEOUT */
    }

    /* If last char to be sent, clear TC flag */
    if (ubSend == (sizeof(aStringToSend) - 1))
    {
      LL_USART_ClearFlag_TC(USARTx_INSTANCE); 
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in DR register */
    LL_USART_TransmitData8(USARTx_INSTANCE, aStringToSend[ubSend++]);
  }

#if (USE_TIMEOUT == 1)
  Timeout = USART_SEND_TIMEOUT_TC_MS;
#endif /* USE_TIMEOUT */

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USARTx_INSTANCE))
  {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    { 
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    } 
#endif /* USE_TIMEOUT */
  }

  ubButtonPress =0;
    
  /* Turn LED2 On at end of transfer : Tx sequence completed successfully */
  LED_On();
#endif
}
#endif

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Enable HSE oscillator */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 400, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

#if 0 
	/* Set systick to 1ms */
  SysTick_Config(100000000 / 1000);
#else
	/* Set systick to 1us */
	SysTick_Config(100000000 / 1000000);
	NVIC_SetPriority(SysTick_IRQn, 0);  
  NVIC_EnableIRQ(SysTick_IRQn);
	
#endif
	
	
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 100000000;
}


/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/
/**
  * @brief  Function to manage Button push
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update button press variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;

}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
	uint8_t ch = LL_USART_ReceiveData8(USARTx_INSTANCE);
	dbg_log_type.uart_rx_buf[uwNbReceivedChars] = ch;
	uwNbReceivedChars = (uwNbReceivedChars + 1) & (DBG_INPUT_BUFFER_SIZE - 1);
	if(ch == '\r'){
    /* Set Buffer swap indication */
    uwBufferReadyIndication = 1;
    uwNbReceivedChars = 0;
		dbg_printf("\n");
	}		
	dbg_printf("%c",ch);	
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t sr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART SR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  sr_reg = LL_USART_ReadReg(USARTx_INSTANCE, SR);
  if (sr_reg & LL_USART_SR_NE)
  {
    /* case Noise Error flag is raised : Clear NF Flag */
    LL_USART_ClearFlag_NE(USARTx_INSTANCE);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    LED_Blinking(LED_BLINK_ERROR);
  }
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
