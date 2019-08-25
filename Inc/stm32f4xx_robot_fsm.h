/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_Tx/Inc/stm32f4xx_robot_fsm.h
  * @author  jszhang
  * @brief   Header for stm32f4xx_robot_fsm.c module
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
#ifndef __STM32F4xx_ROBOT_FSM_H
#define __STM32F4xx_ROBOT_FSM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */


/* Exported macro ------------------------------------------------------------*/
#define    MAX_IROBOT_FSM_STATE_NUM     3
#define    MAX_IROBOT_FSM_EVENT_NUM     4


typedef int (*irobot_fsm_proc)(void *usr_data, uint32_t len);

/****  define state of irobot ****/
enum{
    IROBOT_CELAR_STATE = 0, 
    IROBOT_AVOID_OBSTACLE_STATE,
    IROBOT_CHARGE_STATE,
	  IROBOT_INVAILED_STATE,
};

/****  define all events of irobot ****/
enum{
    IROBOT_NON_OBSTACLE_EVENT = 0,
    IROBOT_OBSTACLES_EVENT,
    IROBOT_CURRENT_IN_EVENT,
    IROBOT_CHARGE_FULL_EVENT,
		IROBOT_INVAILD_EVENT,
};


/*** define the struct of irobot fsm ***/
typedef struct{
    uint32_t state_cnt;
    uint32_t event_cnt;
    uint32_t fsm_state[MAX_IROBOT_FSM_STATE_NUM];
    uint32_t fsm_event[MAX_IROBOT_FSM_EVENT_NUM];
    
    irobot_fsm_proc fsm_proc[MAX_IROBOT_FSM_STATE_NUM][MAX_IROBOT_FSM_EVENT_NUM];
    irobot_fsm_proc fsm_err_proc;
}irobot_fsm_t;


typedef struct{
    uint32_t state;
	  uint32_t event;
}debug_fsm_t;


/*** Export function ***/

int irobot_state_get(uint32_t *state);
int irobot_state_set(uint32_t state);
int irobot_event_get(uint32_t *event);
int irobot_event_set(uint32_t event);

int irobot_add_fsm_proc(uint32_t state, uint32_t event, irobot_fsm_proc fsm_proc);
int irobot_fsm_proc_run(uint32_t state, uint32_t event, void *usr_data, uint32_t len);




#endif /*__STM32F4xx_ROBOT_FSM_H*/













