/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "arm_math.h"
#include "chassis.h"

#define POWER_LIMIT         power_limit
#define WARNING_POWER       power_limit/2  
#define WARNING_POWER_BUFF  100.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

void get_chassis_power_and_buffer(float *power, uint16_t *buffer, uint16_t *power_limit);

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
void chassis_power_control(struct chassis *chassis_power_control)
{
    float chassis_power = 0.0f;
	uint16_t power_limit = 0;
    uint16_t chassis_power_buffer = 0; //buffer Э��������16λ����float���ͳн�
    float total_current_limit = 0.0f;
    float total_current = 0.0f;
//    uint8_t robot_id = get_robot_id();
//    if(toe_is_error(REFEREE_TOE))
//    {
//        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
//    }
//    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
//    {
//        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
//    }
//    else
//    {
	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer, &power_limit);
//	log_printf("%d %d\n", (uint16_t)chassis_power, chassis_power_buffer);

	// power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
	//���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
	if(chassis_power_buffer < WARNING_POWER_BUFF)
	{
		float power_scale;
		if(chassis_power_buffer > 5.0f)
		{
			//scale down WARNING_POWER_BUFF
			//��СWARNING_POWER_BUFF
			power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
		}
		else
		{
			//only left 10% of WARNING_POWER_BUFF
			power_scale = 5.0f / WARNING_POWER_BUFF;
		}
		//scale down
		//��С
		total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
	}
	else
	{
		//power > WARNING_POWER
		//���ʴ���WARNING_POWER
		if(chassis_power > WARNING_POWER)
		{
			float power_scale;
			//power < 80w
			//����С��80w
			if(chassis_power < POWER_LIMIT)
			{
				//scale down
				//��С
				power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
				
			}
			//power > 80w
			//���ʴ���80w
			else
			{
				power_scale = 0.0f;
			}
			
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
		}
		//power < WARNING_POWER
		//����С��WARNING_POWER
		else
		{
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
		}
	}
    //}

    
    total_current = 0.0f;
    //calculate the original motor current set
    //����ԭ����������趨
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_pid[i].out);
    }
    

	
    if(total_current > total_current_limit)
    {
        float current_scale = total_current_limit / total_current;
        chassis_power_control->motor_pid[0].out*=current_scale;
        chassis_power_control->motor_pid[1].out*=current_scale;
        chassis_power_control->motor_pid[2].out*=current_scale;
        chassis_power_control->motor_pid[3].out*=current_scale;
//		log_printf("total_current : %d total_current_limit : %d  current_scale �� %d\n", (uint16_t)total_current, (uint16_t)total_current_limit, (uint16_t)(current_scale*10));
    }
}
