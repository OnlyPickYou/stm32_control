/*****************************************************************************
File name: irobot.c
Description: register irobot state and run robot function 
Author: jszhang@artosyn.cn
Version: v1.0
Date:2019-7-21
*****************************************************************************/
#include "irobot.h"
#include "main.h"

#include <stdio.h>

int User_init(void);



/**
  * @brief  initize robot gpio
  * @note   None
  * @param  None
  * @retval None
  */

int robot_gpio_init(void)
{
	/* Configure GPIOA1 as output mode, PULL-PUSH	*/
  /* Enable the GPIOA Clock */
  TRIG1_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external TRIG */
  LL_GPIO_SetPinMode(TRIG1_GPIO_PORT, TRIG1_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(TRIG1_GPIO_PORT, TRIG1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_HIGH */
  //LL_GPIO_SetPinSpeed(TRIG1_GPIO_PORT, TRIG1_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(TRIG1_GPIO_PORT, TRIG1_PIN, LL_GPIO_PULL_NO);
	
	return 0;
}	


/**
  * @brief  Turn-on UltraSonic TRIG1.
  * @param  None
  * @retval None
  */
void UltraSonic_TRIG1_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(TRIG1_GPIO_PORT, TRIG1_PIN);
}

/**
  * @brief  Turn-off UltraSonic TRIG1.
  * @param  None
  * @retval None
  */
void UltraSonic_TRIG1_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_ResetOutputPin(TRIG1_GPIO_PORT, TRIG1_PIN);
}

void UltraSonic_DetEn(void)
{
	if(TIM1ClockEN_Flag){
		UltraSonic_TRIG1_On();
		LL_mDelay(12);
		UltraSonic_TRIG1_Off();
		TIM1ClockEN_Flag = 0;
	}
}


/**
  * @brief  clear switch in avoid-obstacle state. When the sensor of robot check that there are some obstacles
in front, it will switch state from clear to avoid-obstacle.  
  * @param  usr_data and len
  * @retval None
  */
int irobot_clear_switchin_avoid_obst_state(void *usr_data, uint32_t len)
{
	int ret = _HD_RET_SUCCESS_;
	uint32_t irobot_state = IROBOT_INVAILED_STATE; 
	irobot_state_get(&irobot_state);
	irobot_state_set(IROBOT_AVOID_OBSTACLE_STATE);
	
	dbg_printf("%s in, state %d\n", __func__, irobot_state);	
	return ret;	
}


/**
  * @brief  work in clear state. control the motor run.
  * @param  usr_data and len
  * @retval None
  */
int irobot_work_in_clear_state(void *usr_data, uint32_t len)
{
	
	int ret = _HD_RET_SUCCESS_;
		
	return ret;	
}


/**
  * @brief   avoid-obstacle state switch in clear state. When the sensor of robot check that there are no obstacles
in front, it will switch state from avoid-obstacle to clear.  
  * @param  usr_data and len
  * @retval None
  */
int irobot_avoid_obst_switchin_clear_state(void *usr_data, uint32_t len)
{
	int ret = _HD_RET_SUCCESS_;
	uint32_t irobot_state = IROBOT_INVAILED_STATE; 
	irobot_state_get(&irobot_state);
	irobot_state_set(IROBOT_CELAR_STATE);
	dbg_printf("%s in, state %d\n", __func__, irobot_state);	
	return ret;	
}


/**
  * @brief  work in avoid-obstacle state. control the motor to avoid the obstacle
  * @param  usr_data and len
  * @retval None
  */
int irobot_work_in_avoid_obst_state(void *usr_data, uint32_t len)
{
	int ret = _HD_RET_SUCCESS_;

	return ret;
	
}

/**
  * @brief  avoid-obstacle state switch in charge state. When the sensor of robot check that the current in, 
  it will switch state from avoid-obstacle to charge.  
  * @param  usr_data and len
  * @retval None
  */
int irobot_avoid_obst_switchin_charge_state(void *usr_data, uint32_t len)
{
	int ret = _HD_RET_SUCCESS_;
	uint32_t irobot_state = IROBOT_INVAILED_STATE; 
	irobot_state_get(&irobot_state);
	irobot_state_set(IROBOT_CHARGE_STATE);
	dbg_printf("%s in, state %d\n", __func__,irobot_state);

	return ret;
	
}

/**
  * @brief  work in charge state. shutdown the motor.
  * @param  usr_data and len
  * @retval None
  */

int irobot_work_in_charge_state(void *usr_data, uint32_t len)
{
	int ret = _HD_RET_SUCCESS_;

	return ret;
}

/**
  * @brief  charge state switch in avoid-obstacle state.
  * @param  usr_data and len
  * @retval None
  */
int irobot_charge_switchin_avoid_obst_state(void *usr_data, uint32_t len)
{
	int ret = _HD_RET_SUCCESS_;
	uint32_t irobot_state = IROBOT_INVAILED_STATE;
	irobot_state_get(&irobot_state);	
	irobot_state_set(IROBOT_AVOID_OBSTACLE_STATE);
	
	dbg_printf("%s in,state %d\n", __func__,irobot_state);	
	return ret;
}



int irobot_init(void)
{

	int ret = _HD_RET_SUCCESS_;
	/* robot_uart_init(); */			//初始化串口

	/* robot_adc_init();  */			//初始化adc
	
	/* robot_timer_init();*/			//初始化Timer
	
	robot_gpio_init(); 						//初始化GPIO		

	/* robot_data_init(); */			//初始化数据
	
	return ret;

}


void irobot_register_fsm_handle(void)
{
		/* irobot in clear state, and obstacles_event happened, switch in avoid-obst state */
		irobot_add_fsm_proc(IROBOT_CELAR_STATE, IROBOT_OBSTACLES_EVENT, irobot_clear_switchin_avoid_obst_state );
	
		/* irobot in clear state, and non-obstacles_event happened, always in clear state */
		irobot_add_fsm_proc(IROBOT_CELAR_STATE, IROBOT_NON_OBSTACLE_EVENT, irobot_work_in_clear_state );

		/* irobot in avoid state, and non-obstacles_event happened, switch in clear state */	
		irobot_add_fsm_proc(IROBOT_AVOID_OBSTACLE_STATE, IROBOT_NON_OBSTACLE_EVENT, irobot_avoid_obst_switchin_clear_state );
	
		/* irobot in avoid state, and obstacles_event happened, always in avoid-obst state */	
		irobot_add_fsm_proc(IROBOT_AVOID_OBSTACLE_STATE, IROBOT_OBSTACLES_EVENT, irobot_work_in_avoid_obst_state );
	
		/* irobot in avoid state, and non-obstacles_event happened, switch in clear state */	
		irobot_add_fsm_proc(IROBOT_AVOID_OBSTACLE_STATE, IROBOT_CURRENT_IN_EVENT, irobot_avoid_obst_switchin_charge_state );
	
		/* irobot in avoid state, and obstacles_event happened, always in avoid-obst state */	
		irobot_add_fsm_proc(IROBOT_CHARGE_STATE, IROBOT_CURRENT_IN_EVENT, irobot_work_in_charge_state );
	
		/* irobot in avoid state, and non-obstacles_event happened, switch in clear state */	
		irobot_add_fsm_proc(IROBOT_CHARGE_STATE, IROBOT_CHARGE_FULL_EVENT, irobot_charge_switchin_avoid_obst_state );
		
}


int User_init(void)
{

	int ret = _HD_RET_FAILED_;
	
	
	ret = irobot_init();
	if(ret != _HD_RET_SUCCESS_){
		dbg_printf("irobot_init failed, reason : %d\n", ret);
		return ret;	
	}
	
	irobot_register_fsm_handle();
	
	return ret;

}

int irobot_fsm_handle(uint32_t state, uint32_t event, void *usr_data, uint32_t len)
{
	int ret = _HD_RET_FAILED_;
	ret = irobot_fsm_proc_run(state, event, usr_data, len);
	return ret;
}


void Irobot_mainloop(void)
{
	uint32_t irobot_state = 0;
	uint32_t irobot_event = 0;
	
	UltraSonic_DetEn();
	//LED_Blinking(LED_BLINK_FAST);
	
	irobot_state_get(&irobot_state);
	irobot_event_get(&irobot_event);
  	
	irobot_fsm_handle(irobot_state, irobot_event, NULL, 0);	
}
