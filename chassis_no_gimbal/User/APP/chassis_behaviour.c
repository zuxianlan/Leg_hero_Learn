#include "chassis_behaviour.h"

#include "ChassisL_Task.h"

/**
 * @brief 设置底盘目标值
 * @param
 * @return
 */
void chassis_set_control(chassis_move_t *chassis)
{
    if (chassis == NULL)  {return;}
    if (chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move.Target_Omega = 0.0f;
        chassis_move.Target_Roll = 0.0f;
        chassis_move.Target_Theta = 0.0f;
        chassis->Target_X = chassis->X_filter;

    }
    else if (chassis_mode == CHASSIS_CHECK_IN)
    {
        float vx_channel = 0, vy_channel = 0, vz_channel = 0;
        vx_channel = -chassis->chassis_RC->RC.ch[2];
        // 遥控器摇杆可能存在偏差，死区范围内的输入置零
        vx_channel = Float_Math_Abs(vx_channel) > DR16_Rocker_Dead_Zone ? vx_channel : 0.0f;
        // 设置速度
        chassis_move.Target_Velocity_X = vx_channel * MAX_Velocity_X;
        // 设置偏航角速度
        chassis_move.Target_Omega = 0.0f;
        // 设置偏航目标
        //chassis_move.Target_Yaw = vy_channel;
        // 设置横滚目标
        chassis_move.Target_Roll = 0.0f;
        // 设置 theta 误差目标
        chassis_move.Target_Theta = 0.0f;

        chassis_move.Target_Leg_l = vz_channel * RC_to_Chassis_Leg_Gain;
        chassis_move.Target_Leg_r = vz_channel * RC_to_Chassis_Leg_Gain;
    }
    else if (chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        float vx_channel = 0, vy_channel = 0, vz_channel = 0;
        vx_channel = chassis->chassis_RC->RC.ch[3];
        //vy_channel = chassis->chassis_RC->RC.ch[1];
        vz_channel = chassis->chassis_RC->RC.ch[1];
        // 遥控器摇杆可能存在偏差，死区范围内的输入置零
        vx_channel = Float_Math_Abs(vx_channel) > DR16_Rocker_Dead_Zone ? vx_channel : 0.0f;
        vy_channel = Float_Math_Abs(vy_channel) > DR16_Rocker_Dead_Zone ? vy_channel : 0.0f;
        vz_channel = Float_Math_Abs(vz_channel) > DR16_Rocker_Dead_Zone ? vz_channel : 0.0f;

        // 设置速度
        chassis_move.Target_Velocity_X = vx_channel * MAX_Velocity_X;
        // 设置偏航角速度
        // chassis_move.Target_Omega = -chassis_move.PID_follow_yaw.Out;
        chassis_move.Target_Omega = 0.0f;
        // 设置偏航目标
        //chassis_move.Target_Velocity_Y = vy_channel * MAX_Velocity_Y;
        // 设置横滚目标
        chassis_move.Target_Roll = 0.0f;
        // 设置 theta 误差目标
        chassis_move.Target_Theta = 0.04f;

        chassis_move.Target_Leg_l = vz_channel * RC_to_Chassis_Leg_Gain;
        chassis_move.Target_Leg_r = vz_channel * RC_to_Chassis_Leg_Gain;
    }

}

void FSM_TIM_Calculate_PeriodElapsedCallback_Chassis(FSM_t *FSM)
{
    FSM->Status[FSM->Now_Status].Count_Time++;

    // 自己接着编写状态转移函数
    switch (FSM->Now_Status)
    {
    case NORMAL:
        if (chassis_move.chassis_INS_point->Pitch  >= PI/6.0f && chassis_move.chassis_INS_point->Pitch  <= -PI/6.0f )
        {
            FSM_Set_Status(FSM, OVER_TURN);
        }
        break;
    case OVER_TURN:
        FSM_Set_Status(FSM, OVER_TURNING);
        break;
    case OVER_TURNING:
        if (chassis_move.chassis_INS_point->Pitch  <= PI/10.0f && chassis_move.chassis_INS_point->Pitch  >= -PI/10.0f )
        {
            FSM_Set_Status(FSM, NORMAL);
        }

        break;
    }
}