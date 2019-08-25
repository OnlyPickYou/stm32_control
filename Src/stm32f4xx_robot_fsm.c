/*****************************************************************************
File name: irobot_fsm.c
Description:irobot finite-state machine
Author: jszhang@artosyn.cn
Version: v1.0
Date:2019-7-11
*****************************************************************************/
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_robot_fsm.h"


irobot_fsm_t irobot_fsm;
debug_fsm_t debug_fsm = {IROBOT_CELAR_STATE, IROBOT_NON_OBSTACLE_EVENT};


int irobot_state_get(uint32_t *state)
{
	*state = debug_fsm.state;
	return 0;   
}

int irobot_state_set(uint32_t state)
{
	debug_fsm.state = state;
	return 0;
}

int irobot_event_get(uint32_t *event)
{
	*event = debug_fsm.event;
	return 0;
}

int irobot_event_set(uint32_t event)
{
	debug_fsm.event = event;
	return 0;
}

int irobot_fsm_proc_err(void *user_data, uint32_t len)
{
	return -1;
}


int irobot_add_fsm_proc(uint32_t fsm_state, uint32_t fsm_event, irobot_fsm_proc fsm_proc)
{
    int ret = -1;

    uint8_t state_index;
    uint8_t event_index;

    uint32_t state_exist = 0;
    uint32_t event_exist = 0;

    for(state_index = 0; state_index < irobot_fsm.state_cnt; state_index++)
    {
        if(fsm_state == irobot_fsm.fsm_state[state_index]){
            state_exist = 1;
            break;
        }
    }

    if(0 == state_exist ){

        if(state_index >= MAX_IROBOT_FSM_STATE_NUM){
            dbg_printf("state_index has been exceed, %d\n", fsm_state);
            return ret;
        }
        irobot_fsm.fsm_state[ state_index] = fsm_state;
        irobot_fsm.state_cnt++;
    }
    
    for(event_index = 0; event_index < irobot_fsm.event_cnt; event_index++)
    {
        if(fsm_event == irobot_fsm.fsm_event[event_index]){
            event_exist = 1;
            break;
        }
    }

    if(0 == event_exist){
        if(event_index >= MAX_IROBOT_FSM_EVENT_NUM){
            dbg_printf("event_index has been exceed\n, %d",fsm_event);
            return ret;
        }
        irobot_fsm.fsm_event[event_index] = fsm_event;
        irobot_fsm.event_cnt++;
    }
    irobot_fsm.fsm_proc[state_index][event_index] = fsm_proc;
    dbg_printf("register state %d, event %d, proc 0x%x\n",state_index, event_index, irobot_fsm.fsm_proc[state_index][event_index]);
    return 0;

}

int irobot_fsm_proc_run(uint32_t fsm_state, uint32_t fsm_event, void *usr_data, uint32_t len)
{
    uint8_t state_index =  0;
    uint8_t state_exist = 0;

    uint8_t event_index = 0;
    uint8_t event_exist= 0;

    for(state_index = 0; state_index < irobot_fsm.state_cnt; state_index++)
    {
        if(fsm_state == irobot_fsm.fsm_state[state_index]){
            state_exist = 1;
            break;
        }
    }

    for(event_index = 0; event_index < irobot_fsm.event_cnt; event_index++)
    {
        if(fsm_event == irobot_fsm.fsm_event[event_index]){
            event_exist = 1;
            break;
        }
    }

    if((1 != state_exist) || (1 != event_exist)){
        irobot_fsm_proc_err(usr_data, len);
        return -1;
    }

    if(NULL == irobot_fsm.fsm_proc[state_index][event_index]){
        irobot_fsm_proc_err(usr_data, len);
        return -1;

    }
    irobot_fsm.fsm_proc[state_index][event_index](usr_data, len);
    return 0;



}










