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

#include "can.h"
#include "board.h"
#include "dbus.h"
#include "gimbal.h"
#include "os_timer.h"
#include "gimbal_task.h"
#include "gimbal_cmd.h"
#include "infantry_cmd.h"
#include "event_mgr.h"
#include "event.h"
#include "ramp.h"
#include "easyflash.h"
#include "single_gyro.h"
#include "offline_service.h"

#include "appcfg.h"
#include "iwdg.h"

/* patrol period time (ms) */
#define GIMBAL_PERIOD 5
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 3000

#include "appcfg.h"

#ifdef ICRA2019

struct pid_param yaw_outer_param =
{
    .p = 50.0f,
    .max_out = 2000.0f,
};

struct pid_param yaw_inter_param =
{
    .p = 70.0f,
    .i = 0.2f,
    .max_out = 30000,
    .integral_limit = 3000,
};

struct pid_param pitch_outer_param =
{
    .p = 30.0f,
    .max_out = 2000,
};

struct pid_param pitch_inter_param =
{
    .p = 65.0f,
    .i = 0.2f,
    .max_out = 30000,
    .integral_limit = 3000,
};

#else

struct pid_param yaw_outer_param =
{
    .p = 25.0f,  // 25.0f  11.0f
    .max_out = 2000.0f,
};

struct pid_param yaw_inter_param =
{
    .p = 100.0f,
    .i = 0.3f,
    .max_out = 30000,
    .integral_limit = 3000,
};

struct pid_param pitch_outer_param =
{
    .p = 30.0f,  // 40.0f 21.0f
    .max_out = 2000,
};

struct pid_param pitch_inter_param =
{
    .p = 60.0f,
    .i = 0.1f,
    .max_out = 30000,
    .integral_limit = 3000,
};

#endif



void gimbal_center_adjust(gimbal_t p_gimbal);
void gimbal_normol_handle(struct gimbal *p_gimbal, struct rc_device *p_rc, struct rc_info *p_info);

static void gimbal_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

struct gimbal gimbal;
struct rc_device gimbal_rc;
struct single_gyro single_gyro;

/* gimbal auto adjust */
uint8_t gimbal_adjust_f = 0;
volatile uint32_t pit_time, yaw_time;
uint32_t pit_cnt;
volatile uint16_t yaw_ecd_r, yaw_ecd_l;
volatile uint16_t pit_ecd_r, pit_ecd_l;
struct gimbal_param center_param;

struct ahrs_sensor gimbal_gyro;

float pit_delta, yaw_delta;

/* control ramp parameter */
static ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;

int32_t yaw_angle_fdb_js, yaw_angle_ref_js;
int32_t pit_angle_fdb_js, pit_angle_ref_js;
int32_t yaw_spd_fdb_js, yaw_spd_ref_js;
int32_t pit_spd_fdb_js, pit_spd_ref_js;

uint8_t gimbal_mode = NORMAL_MODE;

static void gimbal_init_handle(gimbal_t p_gimbal);

void gimbal_task(void const *argument)
{
    uint32_t period = osKernelSysTick();

    size_t read_len = 0;
    rc_info_t p_rc_info;

    subscriber_t listSubs;
    subscriber_t nolistSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, gimbal_dr16_data_update);

    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

    rc_device_register(&gimbal_rc, "Gimbal RC");
    p_rc_info = rc_device_get_info(&gimbal_rc);

    single_gyro_init(&single_gyro, "YAW GYRO", 0X401);

    gimbal_cascade_init(&gimbal, "Gimbal",
                        yaw_inter_param,
                        yaw_outer_param,
                        pitch_inter_param,
                        pitch_outer_param,
                        DEVICE_CAN2);


    ef_get_env_blob(GIMBAL_PARAM_KEY, &center_param, sizeof(center_param), &read_len);  // 读取flash中的校准值
#ifdef ENABLE_ADJUST
    if (read_len != sizeof(center_param))  // 没有校准过启动校准模式
    {
        /* no init */
        gimbal_set_work_mode(ADJUST_MODE);
    }
    else
    {
        gimbal_set_offset(&gimbal, center_param.yaw_ecd_center, center_param.pitch_ecd_center);
        gimbal_init_start();  // 云台进入初始化模式
    }
#else
	gimbal_set_offset(&gimbal, center_param.yaw_ecd_center, center_param.pitch_ecd_center);
	gimbal_init_start();  // 云台进入初始化模式
#endif

    gimbal_yaw_disable(&gimbal);
    gimbal_pitch_disable(&gimbal);

    while (1)
    {
#ifndef ENABLE_ADJUST
		HAL_IWDG_Refresh(&hiwdg);  // 喂狗
#endif
		
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);
        /* gyro data update */
        EventMsgGetLast(&nolistSubs, AHRS_MSG, &gimbal_gyro, NULL);
#ifdef ICRA2019
        gimbal_pitch_gyro_update(&gimbal, gimbal_gyro.roll * RAD_TO_DEG);
        gimbal_rate_update(&gimbal, gimbal_gyro.gz * RAD_TO_DEG, -gimbal_gyro.gx * RAD_TO_DEG);
#else
        gimbal_pitch_gyro_update(&gimbal, gimbal_gyro.pitch * RAD_TO_DEG);
        gimbal_rate_update(&gimbal, gimbal_gyro.gz * RAD_TO_DEG, gimbal_gyro.gy * RAD_TO_DEG);
#endif
        switch (gimbal_mode)
        {
        case NORMAL_MODE:
        {
            gimbal_normol_handle(&gimbal, &gimbal_rc, p_rc_info); // 处理normal模式
            gimbal_cascade_calculate(&gimbal);
        }
        break;
        case ADJUST_MODE:
            gimbal_center_adjust(&gimbal);
            break;
        case INIT_MODE:  // 初始化模式 将云台角度调整至中间位置
        {
            gimbal_init_handle(&gimbal);
            gimbal_cascade_calculate(&gimbal);
        }
        case YAW_DEBUG_MODE:
            ;
            break;
        case PITCH_DEBUG_MODE:
            ;
            break;
        };

        yaw_angle_fdb_js = gimbal.yaw_outer_pid.get * 1000;
        yaw_angle_ref_js = gimbal.yaw_outer_pid.set * 1000;
        pit_angle_fdb_js = gimbal.yaw_inter_pid.get * 1000;
        pit_angle_ref_js = gimbal.yaw_inter_pid.set * 1000;

        yaw_spd_fdb_js = gimbal.pitch_outer_pid.get * 1000;
        yaw_spd_ref_js = gimbal.pitch_outer_pid.set * 1000;
        pit_spd_fdb_js = gimbal.pitch_inter_pid.get * 1000;
        pit_spd_ref_js = gimbal.pitch_inter_pid.set * 1000;

        osDelayUntil(&period, GIMBAL_PERIOD);
    }
}

void gimbal_gyro_yaw_update(uint16_t std_id, uint8_t *data)
{
    single_gyro_update(&single_gyro, std_id, data);
    gimbal_yaw_gyro_update(&gimbal, single_gyro.yaw_gyro_angle + gimbal.ecd_angle.yaw);
}

/**
  * @brief  gimbal cold boot
  * @param
  * @retval void
  */
void gimbal_init_start(void)
{
    ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME / GIMBAL_PERIOD);
    ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME / GIMBAL_PERIOD);
    gimbal_set_work_mode(INIT_MODE);
}

#define ANGLE_ABS(x) ((x) > 0 ? (x) : (-(x)))

/**
  * @brief  init status handle
  * @param
  * @retval void
  */
static void gimbal_init_handle(gimbal_t p_gimbal)
{
    gimbal_set_pitch_mode(p_gimbal, ENCODER_MODE);
    gimbal_set_yaw_mode(p_gimbal, ENCODER_MODE);
    gimbal_yaw_disable(p_gimbal);
    gimbal_set_pitch_angle(p_gimbal, p_gimbal->ecd_angle.pitch * (1 - ramp_v0_calculate(&pitch_ramp)));

    if ((p_gimbal->ecd_angle.pitch != 0) && (p_gimbal->ecd_angle.yaw != 0))
    {
        if (fabsf(p_gimbal->ecd_angle.pitch) < 2.0f)
        {
            gimbal_yaw_enable(p_gimbal);
            gimbal_set_yaw_angle(p_gimbal, p_gimbal->ecd_angle.yaw * (1 - ramp_v0_calculate(&yaw_ramp)), 0);
            if (fabsf(p_gimbal->ecd_angle.yaw) < 1.2f)
            {
                gimbal_set_work_mode(NORMAL_MODE);
            }
        }
    }
}

/**
* @brief  work mode: normal/adjust
  * @param
  * @retval void
  */
void gimbal_set_work_mode(uint8_t mode)
{
    gimbal_mode = mode;
}

uint8_t gimbal_get_work_mode(void)
{
    return gimbal_mode;
}

/**
  * @brief  normal status handle
  * @param
  * @retval void
  */
void gimbal_normol_handle(struct gimbal *p_gimbal, struct rc_device *p_rc, struct rc_info *p_info)
{
#ifdef ICRA2019
    p_info->ch4 = -p_info->ch4;
#endif
    /* follow mode */
    if (rc_device_get_state(p_rc, RC_S2_UP) == E_OK)  // 右拨杆上
    {
        pit_delta = -(float)p_info->ch4 * 0.0015f;
        yaw_delta = -(float)p_info->ch3 * 0.0015f;
		
		// 键鼠映射
        if (p_info->mouse.x) {
            yaw_delta = -(float)p_info->mouse.x * 0.009f;
        }
        if (p_info->mouse.y) {
            pit_delta = -(float)p_info->mouse.y * 0.012f;
        }
//		if (p_info->kb.key_code & CHASSIS_SCOPPERIL_KEY)//小陀螺
//		{
//			// 云台固定角度不动
//			gimbal_set_yaw_mode(p_gimbal, GYRO_MODE);
//			yaw_delta = -(float)p_info->ch3 * 0.0015f;
//			pit_delta = -(float)p_info->ch4 * 0.0015f;
//			if (p_info->mouse.x) {
//				yaw_delta = -(float)p_info->mouse.x * 0.009f;
//			}
//			if (p_info->mouse.y) {
//				pit_delta = -(float)p_info->mouse.y * 0.012f;
//			}
//		}
		if (p_info->mouse.r == 0) {
			gimbal_set_yaw_mode(p_gimbal, GYRO_MODE); // 底盘跟随云台 云台根据遥控器的值进行角度控制
			gimbal_set_pitch_delta(p_gimbal, pit_delta);
			gimbal_set_yaw_delta(p_gimbal, yaw_delta);
		}
    }

    /* encoder mode */
    if (rc_device_get_state(p_rc, RC_S2_MID) == E_OK)  // 右拨杆中
    {
        gimbal_set_yaw_mode(p_gimbal, ENCODER_MODE);  // 云台跟随底盘 云台根据电机编码器控制
        pit_delta = -(float)p_info->ch4 * 0.0015f;
        gimbal_set_pitch_delta(p_gimbal, pit_delta);

		
        
		// 云台固定角度不动
//		gimbal_set_yaw_mode(p_gimbal, GYRO_MODE);
//		yaw_delta = -(float)p_info->ch3 * 0.0015f;
//		pit_delta = -(float)p_info->ch4 * 0.0015f;
//		gimbal_set_pitch_delta(p_gimbal, pit_delta);
//		gimbal_set_yaw_delta(p_gimbal, yaw_delta);
		

        if (rc_device_get_state(p_rc, RC_S2_UP2MID) == E_OK)  // 从上切换到中间时云台回正
        {
            gimbal_set_yaw_angle(p_gimbal, 0, 0);
        }
    }

    if (rc_device_get_state(p_rc, RC_S2_DOWN2MID) == E_OK)
    {
        gimbal_set_yaw_angle(p_gimbal, 0, 0);
    }

    if (rc_device_get_state(p_rc, RC_S2_DOWN) == E_OK || p_info->mouse.r)  // 右拨杆下
    {
        set_gimbal_sdk_mode(GIMBAL_SDK_ON);
        gimbal_set_yaw_mode(p_gimbal, ENCODER_MODE);  // 自动 使用encoder控制
        offline_event_enable(OFFLINE_MANIFOLD2_HEART);
        offline_event_enable(OFFLINE_CONTROL_CMD);
    }
    else
    {
        /* disable sdk */
        set_gimbal_sdk_mode(GIMBAL_SDK_OFF);
        offline_event_disable(OFFLINE_MANIFOLD2_HEART);
        offline_event_disable(OFFLINE_CONTROL_CMD);

        offline_event_enable(OFFLINE_GIMBAL_PITCH);
        offline_event_enable(OFFLINE_GIMBAL_YAW);
        offline_event_enable(OFFLINE_GIMBAL_TURN_MOTOR);
    }
}

/**
  * @brief  encoder center value auto initialize.
  * @param
  * @retval void
  */
void gimbal_center_adjust(gimbal_t p_gimbal)
{
    struct motor_device *p_motor;

    p_motor = &p_gimbal->pitch_motor;

    /* pitch */
    {
        pit_time = get_time_ms();
        while (get_time_ms() - pit_time <= 2000)
        {
            motor_set_current(p_motor, 8000);
            pit_ecd_l = p_motor->data.ecd;
            HAL_Delay(2);
        }

        pit_time = HAL_GetTick();
        while (HAL_GetTick() - pit_time <= 2000)
        {
            motor_set_current(p_motor, -8000);
            pit_ecd_r = p_motor->data.ecd;
            HAL_Delay(2);
        }

        if (pit_ecd_l > pit_ecd_r)
        {
            center_param.pitch_ecd_center = (pit_ecd_l + pit_ecd_r) / 2;
        }
        else
        {
            if ((pit_ecd_l + pit_ecd_r) / 2 > 4096)
            {
                center_param.pitch_ecd_center = (pit_ecd_l + pit_ecd_r) / 2 - 4096;
            }
            else
            {
                center_param.pitch_ecd_center = (pit_ecd_l + pit_ecd_r) / 2 + 4096;
            }
        }
    }

    p_motor = &p_gimbal->yaw_motor;
    /* yaw */
    {
        yaw_time = get_time_ms();
        while (get_time_ms() - yaw_time <= 3000)
        {
            motor_set_current(p_motor, 6000);
            yaw_ecd_l = p_motor->data.ecd;
            HAL_Delay(2);
        }

        yaw_time = HAL_GetTick();
        while (HAL_GetTick() - yaw_time <= 3000)
        {
            motor_set_current(p_motor, -6000);
            yaw_ecd_r = p_motor->data.ecd;
            HAL_Delay(2);
        }

        if (yaw_ecd_l > yaw_ecd_r)
        {
            center_param.yaw_ecd_center = (yaw_ecd_l + yaw_ecd_r) / 2;
        }
        else
        {
            if ((yaw_ecd_l + yaw_ecd_r) / 2 > 4096)
            {
                center_param.yaw_ecd_center = (yaw_ecd_l + yaw_ecd_r) / 2 - 4096;
            }
            else
            {
                center_param.yaw_ecd_center = (yaw_ecd_l + yaw_ecd_r) / 2 + 4096;
            }
        }
    }

    ef_set_env_blob(GIMBAL_PARAM_KEY, &center_param, sizeof(center_param));

    /* reboot */
    __disable_irq();
    NVIC_SystemReset();
    while (1)
        ;
}

struct gimbal *get_gimbal(void)
{
    return &gimbal;
}

/**
  * @brief  subscrib dr16 event, update
  * @param
  * @retval void
  */
static void gimbal_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&gimbal_rc, pMsgData);
}
