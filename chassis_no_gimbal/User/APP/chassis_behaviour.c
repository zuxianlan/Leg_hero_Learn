#include "chassis_behaviour.h"
#include "ChassisL_Task.h"

void FSM_TIM_Calculate_PeriodElapsedCallback_Chassis(FSM_t *FSM);

/**
 * @brief 选择底盘模式
 *
 * @param chassis 底盘全局数据指针
 * @return
 */
void chassis_mode_set(chassis_move_t *chassis)
{
    if (chassis == NULL) {return;}
    if (switch_is_up(chassis->chassis_RC->RC.sw[3]))
    {
        chassis_mode = CHASSIS_ZERO_FORCE;
    }
    else
    {
        FSM_TIM_Calculate_PeriodElapsedCallback_Chassis(&chassis->FSM);
        if (switch_is_mid(chassis->chassis_RC->RC.sw[3]) && switch_is_up(chassis->chassis_RC->RC.sw[1]))
        {
            chassis_mode = CHASSIS_CHECK_IN;
        }
        else if (switch_is_mid(chassis->chassis_RC->RC.sw[3]) && switch_is_down(chassis->chassis_RC->RC.sw[1]))
        {
            chassis_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
        }
    }

}

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

    const float theta_l = chassis_move.left_leg.theta;
    const float theta_r = chassis_move.right_leg.theta;
    const float L0_l    = chassis_move.left_leg.L0;
    const float L0_r    = chassis_move.right_leg.L0;
    const float pitch   = chassis_move.pitch;
    // 自己接着编写状态转移函数
    switch (FSM->Now_Status)
    {
    case NORMAL:
        if (theta_l > 0.9f || theta_r > 0.9f || theta_l < -0.9f || theta_r < -0.9f)
        {
            FSM_Set_Status(FSM, OVER_TURN);
        }
        break;
    case OVER_TURN:
        if (Float_Math_Abs(theta_l - theta_r) < 0.3f)
        {
            if (theta_l >= 0.7f || theta_r >= 0.7f)
            {
                FSM_Set_Status(FSM, OVER_TURNING);
            }
        }
        // else
        // {
        //     /* 双腿劈开：一腿 ≥ 0.9 而另一腿仍在 [-0.8, 0.4] */
        //     if ((theta_l >= 0.9f && theta_r <= 0.4f && theta_r >= -0.8f) ||
        //         (theta_r >= 0.9f && theta_l <= 0.4f && theta_l >= -0.8f))
        //     {
        //         FSM_Set_Status(FSM, OVER_TURNING);
        //     }
        // }
       break;
    case OVER_TURNING:
        if (Float_Math_Abs(pitch) <= PI/4.0f && theta_l <= 0.2f && theta_l >= -0.8f && theta_r <= 0.2f && theta_r >= -0.8f && L0_l < 0.2f && L0_r < 0.2f)
        {
            FSM_Set_Status(FSM, NORMAL);
        }
        break;

    default:
        break;
    }
}

void chassis_control_left_leg(float target)
{
    if(target != 0.0f)
    {
        // LQR力矩计算
        chassis_move.T_wl = 0.0f;
        chassis_move.left_leg.Tp = (chassis_move.left_leg.d_theta - target) * Fitting_K[2][5];
        chassis_move.left_leg.F0 = 0.0f;
        VMC_Calc_2(&chassis_move.left_leg);
    }
    else
    {
        chassis_move.T_wl = 0.0f;
        chassis_move.left_leg.Tp = 0.0f;
        chassis_move.left_leg.F0 = 0.0f;
        VMC_Calc_2(&chassis_move.left_leg);
    }
}
void chassis_control_right_leg(float target)
{
    if(target != 0.0f)
    {
        // LQR力矩计算
        chassis_move.T_wr = 0.0f;
        chassis_move.right_leg.Tp = (chassis_move.right_leg.d_theta - target) * Fitting_K[3][7];
        chassis_move.right_leg.F0 = 0.0f;
        VMC_Calc_2(&chassis_move.right_leg);
    }
    else
    {
        chassis_move.T_wr = 0.0f;
        chassis_move.right_leg.Tp = 0.0f;
        chassis_move.right_leg.F0 = 0.0f;
        VMC_Calc_2(&chassis_move.right_leg);
    }
}

void chassis_over_turn_mod(chassis_move_t *chassis)
{
    chassis->Target_Velocity_X = 0.0f;
    chassis->Target_X          = 0.0f;
    chassis->Target_Omega      = 0.0f;
    chassis->Target_Roll       = 0.0f;
    chassis->Target_Theta      = 0.0f;

    if (chassis->left_leg.theta < -0.9f || chassis->left_leg.theta > 1.4f)
    {
        chassis_control_left_leg(-4.0f);
    }
    else
    {
        chassis_control_left_leg(0.0f);
    }
    if (chassis->right_leg.theta < -0.9f || chassis->right_leg.theta > 1.4f)
    {
        chassis_control_right_leg(-4.0f);
    }
    else
    {
        chassis_control_right_leg(0.0f);
    }

}
void chassis_control_leg_pid_loop(chassis_move_t *chassis)
{
    /* 轮子零力矩 */
    chassis->T_wl = 0.0f;
    chassis->T_wr = 0.0f;

    /* 腿长目标直接压到最短 —— 这就是"收腿" */
    chassis->PID_legL_Position.Target = MIN_LEG_LENGTH;
    chassis->PID_legL_Position.Now    = chassis->left_leg.L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legL_Position);

    chassis->PID_legR_Position.Target = MIN_LEG_LENGTH;
    chassis->PID_legR_Position.Now    = chassis->right_leg.L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legR_Position);

    /* 收腿阻尼 */
    chassis->PID_legL_Velocity.Target = 0.0f;
    chassis->PID_legL_Velocity.Now    = chassis->left_leg.d_L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legL_Velocity);

    chassis->PID_legR_Velocity.Target = 0.0f;
    chassis->PID_legR_Velocity.Now    = chassis->right_leg.d_L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legR_Velocity);

    /* 髋关节常值力矩把 θ 往回收 */
    chassis->PID_tp.Target = 0.0f;
    chassis->PID_tp.Now    = chassis->theta_err;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_tp);

    chassis->PID_tp_omega.Target = chassis->PID_tp.Out;
    chassis->PID_tp_omega.Now    = chassis->d_theta_err;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_tp_omega);

    // chassis->left_leg.Tp  =  4.0f - chassis->PID_tp_omega.Out;
    // chassis->right_leg.Tp =  4.0f + chassis->PID_tp_omega.Out;
    chassis->left_leg.Tp  =  4.0f;
    chassis->right_leg.Tp =  4.0f;

    /* 沿腿力：腿长 PID + 弹簧补偿（和 Follow 模式一致） */
    chassis->left_leg.F0  =  chassis->PID_legL_Position.Out + chassis->PID_legL_Velocity.Out- chassis->spring_force_l + 10.0f;
    chassis->right_leg.F0 = -chassis->PID_legR_Position.Out - chassis->PID_legR_Velocity.Out+ chassis->spring_force_r;

    VMC_Calc_2(&chassis->left_leg);
    VMC_Calc_2(&chassis->right_leg);
}