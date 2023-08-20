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

#include "dbus.h"
#include "chassis_task.h"
#include "chassis_cmd.h"
#include "os_timer.h"
#include "infantry_cmd.h"
#include "board.h"
#include "event_mgr.h"
#include "event.h"
#include "chassis.h"
#include "offline_service.h"
#include <stdbool.h>

#include "appcfg.h"
#include "iwdg.h"


struct pid_param chassis_motor_param =
{
    .p = 6.5f,
    .i = 0.1f,
    .max_out = 15000,
    .integral_limit = 500,
};

static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);
static int32_t chassis_angle_broadcast(void *argv);

struct chassis chassis;
struct rc_device chassis_rc;
struct ahrs_sensor chassis_gyro;

bool shift_flag = 0;

/* chassis speed */
static float vx, vy, wz;
static float vx_set, vy_set, wz_set;
static int set_chassis_speed_xy;
static int set_chassis_speed_z;
static uint8_t scopperil_key_state;
static uint8_t last_scopperil_key_state;
static bool isScopperil = 0;

float angle_buffer[100];


//#pragma pack(1)
//static float vx_key , vy_key; // 不知道为什么，使用这个变量会导致程序无法运行（遥控器一直掉线）
//#pragma pack()
#define SET_CHASSIS_VX_SPEED_KEY 600.0f
#define SET_CHASSIS_VY_SPEED_KEY 600.0f
#define TIMEOUT 100

/* fllow control */
struct pid pid_follow = {0};
float follow_relative_angle;
float gimbal_angle_from_gimbal;

void chassis_task(void const *argument)
{
    rc_info_t p_rc_info;

    subscriber_t listSubs;
    subscriber_t nolistSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, chassis_dr16_data_update);

    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

    rc_device_register(&chassis_rc, "Chassis RC");
    p_rc_info = rc_device_get_info(&chassis_rc);

    chassis_pid_init(&chassis, "Chassis", chassis_motor_param, DEVICE_CAN2);

    soft_timer_register((soft_timer_callback)chassis_pid_calculate, (void *)&chassis, 5);
    soft_timer_register((soft_timer_callback)chassis_angle_broadcast, (void *)NULL, 10);

    pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

    while (1)
    {
#ifndef ENABLE_ADJUST
		HAL_IWDG_Refresh(&hiwdg);  // 喂狗
#endif
		
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);
        /* gyro data update */
        EventMsgGetLast(&nolistSubs, AHRS_MSG, &chassis_gyro, NULL);

        chassis_gyro_updata(&chassis, chassis_gyro.yaw * RAD_TO_DEG, chassis_gyro.gz * RAD_TO_DEG);

        if (rc_device_get_state(&chassis_rc, RC_S2_UP) == E_OK)  // 右拨杆上
        {
            wz = -pid_calculate(&pid_follow, follow_relative_angle, 0);  // 底盘跟随云台
			
			// 需要把遥控器控制注释掉
			if (p_rc_info->kb.key_code & CHASSIS_FRONT_KEY) {
				if(vx_set < set_chassis_speed_xy) vx_set += CHASSIS_SPEED_ACC;
				else vx_set = set_chassis_speed_xy;
            }
            else if (p_rc_info->kb.key_code & CHASSIS_BACK_KEY) {
				if(vx_set > -set_chassis_speed_xy) vx_set -= CHASSIS_SPEED_ACC;
				else vx_set = -set_chassis_speed_xy;
            }
			else {
				if(vx_set > 0) vx_set -= CHASSIS_SPEED_ACC;
				else if(vx_set < 0) vx_set += CHASSIS_SPEED_ACC;
				else vx_set = 0;
			}

            if (p_rc_info->kb.key_code & CHASSIS_LEFT_KEY) {
				if(vy_set < set_chassis_speed_xy) vy_set += CHASSIS_SPEED_ACC;
				else vy_set = set_chassis_speed_xy;
            }
            else if (p_rc_info->kb.key_code & CHASSIS_RIGHT_KEY) {
				if(vy_set > -set_chassis_speed_xy) vy_set -= CHASSIS_SPEED_ACC;
				else vy_set = -set_chassis_speed_xy;
            }
			else {
				if(vy_set > 0) vy_set -= CHASSIS_SPEED_ACC;
				else if(vy_set < 0) vy_set += CHASSIS_SPEED_ACC;
				else vy_set = 0;
			}
			
			
			if (p_rc_info->kb.key_code & CHASSIS_SCOPPERIL_KEY) { //小陀螺
				isScopperil = 1;
			}
			
			if (p_rc_info->kb.key_code & CHASSIS_SCOPPERIL_DISABLE_KEY) { //小陀螺
				isScopperil = 0;
			}
			
			
			
			if (p_rc_info->kb.key_code & CHASSIS_QUICKEN_KEY) { //加速
				shift_flag = 1;
				set_chassis_speed_xy = SET_CHASSIS_VXY_SPEED_UP;
				set_chassis_speed_z = SET_CHASSIS_WZ_SPEED_UP;
			}
			else {
				shift_flag = 0;
				set_chassis_speed_xy = SET_CHASSIS_VXY_SPEED;
				set_chassis_speed_z = SET_CHASSIS_WZ_SPEED;
			}
			
			// 坐标变换，以云台方向为前进方向
			float theta;
			theta = follow_relative_angle / 180 * 3.1415;
			vy = ((float)vy_set*cos(theta) + (float)vx_set*sin(theta));
			vx = ((float)vy_set*-sin(theta) + (float)vx_set*cos(theta));
			
			if(isScopperil) wz = set_chassis_speed_z;
			
            chassis_set_offset(&chassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
            chassis_set_acc(&chassis, 0, 0, 0);
            chassis_set_speed(&chassis, vx, vy, wz);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID) == E_OK)  // 右拨杆中
        {
            vx = (float)p_rc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
            vy = -(float)p_rc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
            wz = -(float)p_rc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;  // 云台跟随底盘
            chassis_set_offset(&chassis, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
            chassis_set_speed(&chassis, vx, vy, wz);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID2DOWN) == E_OK)
        {
            chassis_set_speed(&chassis, 0, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID2UP) == E_OK)
        {
            chassis_set_speed(&chassis, 0, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_DOWN) == E_OK)  // 右拨杆下
        {
            set_chassis_sdk_mode(CHASSIS_SDK_ON);
            offline_event_enable(OFFLINE_MANIFOLD2_HEART);
            offline_event_enable(OFFLINE_CONTROL_CMD);

            if ((p_rc_info->ch1 < -400) && (p_rc_info->ch2 < -400) && (p_rc_info->ch3 > 400) && (p_rc_info->ch4 < -400))
            {
                static int cnt = 0;
                cnt++;
                /* 2 second */
                if (cnt > 400)
                {
                    motor_auto_set_id(DEVICE_CAN2);
                }
            }
        }
        else
        {
            /* disable sdk */
            set_chassis_sdk_mode(CHASSIS_SDK_OFF);
            offline_event_disable(OFFLINE_MANIFOLD2_HEART);
            offline_event_disable(OFFLINE_CONTROL_CMD);

            offline_event_enable(OFFLINE_CHASSIS_MOTOR1);
            offline_event_enable(OFFLINE_CHASSIS_MOTOR2);
            offline_event_enable(OFFLINE_CHASSIS_MOTOR3);
            offline_event_enable(OFFLINE_CHASSIS_MOTOR4);
        }

        osDelay(5);
    }
}

/**
  * @brief  send chassis angle to gimbal
  * @param
  * @retval void
  */
int32_t chassis_angle_broadcast(void *argv)
{
    int32_t s_yaw, s_yaw_rate;

    s_yaw = chassis.mecanum.gyro.yaw_gyro_angle * 1000;
    s_yaw_rate = chassis.mecanum.gyro.yaw_gyro_rate * 1000;

    uint8_t data[8];
    data[0] = s_yaw >> 24;
    data[1] = s_yaw >> 16;
    data[2] = s_yaw >> 8;
    data[3] = s_yaw;
    data[4] = s_yaw_rate >> 24;
    data[5] = s_yaw_rate >> 16;
    data[6] = s_yaw_rate >> 8;
    data[7] = s_yaw_rate;

    can1_std_transmit(0x401, data, 8);
    return 0;
}

struct chassis *get_chassis(void)
{
    return &chassis;
}

/**
  * @brief  subscrib dr16 event, update
  * @param
  * @retval void
  */
static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&chassis_rc, pMsgData);
}

/**
  * @brief  follow mode angle update
  * @param
  * @retval void
  */
int32_t follow_angle_info_rcv(uint8_t *buff, uint16_t len)
{
    struct cmd_gimbal_info *info;
    info = (struct cmd_gimbal_info *)buff;
    follow_relative_angle = info->yaw_ecd_angle / 10.0f;
    gimbal_angle_from_gimbal = info->yaw_gyro_angle / 10.0f;
    offline_event_time_update(OFFLINE_GIMBAL_INFO);
    return 0;
}

void set_follow_relative(float val)
{
    follow_relative_angle = val;
}
