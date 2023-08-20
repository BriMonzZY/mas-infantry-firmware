/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "sys.h"
#include "shoot.h"
#include "dbus.h"
#include "shoot.h"
#include "shoot_task.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"

#include "tim.h"

struct pid_param turn_motor_param =
{
    .p = 10.0f,
    .i = 0.3f,
    .max_out = 30000,
    .integral_limit = 10000,
};

static void shoot_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

struct shoot shoot;
struct rc_device shoot_rc;

uint8_t toggle = 0;

int32_t shoot_firction_toggle(shoot_t p_shoot);

void shoot_task(void const *argument)
{
    uint32_t shoot_time = 0;

    subscriber_t listSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, shoot_dr16_data_update);

    rc_device_register(&shoot_rc, "Shoot RC");

    soft_timer_register((soft_timer_callback)shoot_pid_calculate, (void *)&shoot, 5);

    shoot_pid_init(&shoot, "Shoot", turn_motor_param, DEVICE_CAN2);

    rc_info_t p_info;
    p_info = rc_device_get_info(&shoot_rc);

    while (1)
    {
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);

        if (rc_device_get_state(&shoot_rc, RC_S1_MID2UP) == E_OK)
        {
            shoot_firction_toggle(&shoot);
        }
		
		if (rc_device_get_state(&shoot_rc, RC_S1_DOWN) == E_OK)  // 左边拨杆下 弹速慢
        {
			if(toggle) {
				shoot_set_fric_speed(&shoot, 1480, 1480);
			}
        }
		if (rc_device_get_state(&shoot_rc, RC_S1_MID) == E_OK)
        {
            if(toggle) {
				shoot_set_fric_speed(&shoot, 1800, 1800);
			}
        }
		

//        // 单发、连发
//        if (p_info->wheel == 0) {
//            shoot.target.motor_speed = 0;
//            shoot.state = SHOOT_READY;
//        }
//        else {
//            shoot.target.motor_speed = shoot.param.turn_speed;
//            shoot.state = SHOOT_INIT;
//            if (shoot.cmd == SHOOT_ONCE_CMD) {
//                shoot.shoot_num++;
//                shoot.cmd = SHOOT_STOP_CMD;
//            }
//        }
		
		if (p_info->mouse.l || p_info->wheel  != 0) {  // 正转(顺时针)
			shoot.target.motor_speed = shoot.param.turn_speed;
            shoot.state = SHOOT_INIT;
            if (shoot.cmd == SHOOT_ONCE_CMD) {
                shoot.shoot_num++;
                shoot.cmd = SHOOT_STOP_CMD;
            }
        }
		else {
			shoot.target.motor_speed = 0;
            shoot.state = SHOOT_READY;
		}

        //        if (rc_device_get_state(&shoot_rc, RC_S1_MID2DOWN) == E_OK)
        //        {
        //            shoot_set_cmd(&shoot, SHOOT_ONCE_CMD, 1);
        //            shoot_time = get_time_ms();
        //        }

        //        if (rc_device_get_state(&shoot_rc, RC_S2_DOWN) != E_OK)
        //        {
        //            if (rc_device_get_state(&shoot_rc, RC_S1_DOWN) == E_OK)
        //            {
        //                if (get_time_ms() - shoot_time > 2500)
        //                {
        //                    shoot_set_cmd(&shoot, SHOOT_CONTINUOUS_CMD, 0);
        //                }
        //            }

        //            if (rc_device_get_state(&shoot_rc, RC_S1_MID) == E_OK)
        //            {
        //                shoot_set_cmd(&shoot, SHOOT_STOP_CMD, 0);
        //            }
        //        }
        osDelay(5);
    }
}

int32_t shoot_firction_toggle(shoot_t p_shoot)
{
//    static uint8_t toggle = 0;
    if (toggle)
    {
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // 关闭激光
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
        shoot_set_fric_speed(p_shoot, 1000, 1000);
    }
    else
    {
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1500);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  // 启动激光
        shoot_set_fric_speed(p_shoot, 1800, 1800);
    }
    toggle = ~toggle;
    return 0;
}

struct shoot *get_shoot(void)
{
    return &shoot;
}

/**
  * @brief  subscrib dr16 event, update
  * @param
  * @retval void
  */
static void shoot_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&shoot_rc, pMsgData);
}
