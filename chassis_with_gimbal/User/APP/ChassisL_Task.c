/**
  ******************************************************************************
  * @file           : ChassisL_Task.c
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/8/6
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "ChassisL_Task.h"
#include "fdcan.h"
#include "CAN_receive.h"
#include "Host_Comm_Task.h"
#include "VMC&LQR_Calc.h"
#include "motor_dji.h"
#include "motor_dm.h"
#include "Can_Comm_Task.h"
#include "chassis_behaviour.h"
#include "chassis_kalman.h"
#include "user_lib.h"

/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/
chassis_move_t chassis_move;
chassis_mode_e chassis_mode;
const uint32_t chassis_time = 1;
extern float mpc_out;

/* Function Declaration ------------------------------------------------------*/
static void chassis_init(chassis_move_t * chassis_move_init);
void Chassis_Feedback_Update(chassis_move_t *chassis);
static void chassis_control_loop(chassis_move_t *chassis);
static void chassis_normal_pid_loop(chassis_move_t *chassis);
static void chassis_lqr_calc_to_motor(chassis_move_t *chassis);
static void chassis_output_to_motor(chassis_move_t *chassis);
void Chassis_Motor_Status_PeriodElapsedCallback(chassis_move_t *chassis);
void chassis_motor_keep_alive(chassis_move_t *chassis);
static float Max_Output(float num,float max);
void chassis_stop_state_machine(chassis_move_t *chassis);

uint8_t cap[8] = {0};
float Fitting_K[4][10] = {}; // 10 维 LQR 增益矩阵（4输入×10状态，由 P[40][6] 系数表按左右腿长实时重建）
int a=0;
/* Function ------------------------------------------------------------------*/
/**
 * @brief 底盘初始化
 *
 * @param
 * @return
 */
void ChassisL_Task(void)
{
    while (INS.ins_flag == 0)
    {
        osDelay(1);
    }
    chassis_init(&chassis_move);

    while (1)
    {
        chassis_mode_set(&chassis_move); //目标设置
        chassis_set_control(&chassis_move);
        chassis_stop_state_machine(&chassis_move);
        Chassis_Feedback_Update(&chassis_move); //底盘数据更新
        chassis_control_loop(&chassis_move); //PID计算,lqr计算

        chassis_output_to_motor(&chassis_move);
        Chassis_Motor_Status_PeriodElapsedCallback(&chassis_move);
        CAP_AddTxPacket(&chassis_move.Super_Cap_Tx, cap);
        TIM_CAN_PeriodElapsedCallback(); //轮电机，超电数据发送

        osDelay(1);
    }
}

/**
 * @brief 左腿初始化
 *
 * @param
 * @return
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    //remote_control_init();
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->chassis_INS_point = get_INS_point();

    /* 前
     * 0 2
     * 1 3
     */
    Motor_DM_Normal_Init(&chassis_move_init->Motor_Joint[3], &hfdcan1, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT,
                         1 * PI, 45.0f, 54.0f, 3.0f);
    Motor_DM_Normal_Init(&chassis_move_init->Motor_Joint[2], &hfdcan1, 0x22, 0x02, Motor_DM_Control_Method_NORMAL_MIT,
                         1 * PI, 45.0f, 54.0f, 3.0f);
    Motor_DM_Normal_Init(&chassis_move_init->Motor_Joint[1], &hfdcan1, 0x33, 0x03, Motor_DM_Control_Method_NORMAL_MIT,
                         1 * PI, 45.0f, 54.0f, 3.0f);
    Motor_DM_Normal_Init(&chassis_move_init->Motor_Joint[0], &hfdcan1, 0x44, 0x04, Motor_DM_Control_Method_NORMAL_MIT,
                         1 * PI, 45.0f, 54.0f, 3.0f);
    Motor_DM_Normal_Init(&chassis_move_init->Motor_Yaw, &hfdcan3, 0x55, 0x05, Motor_DM_Control_Method_NORMAL_MIT, 1 * PI,30.0f, 10.0f, 3.0f);

    Motor_C620_Init(&chassis_move_init->Motor_Wheel[0], &hfdcan2, CAN_Motor_ID_0x202, Motor_DJI_Control_Method_CURRENT,
                    Motor_DJI_Power_Limit_Status_DISABLE, 20.0f);
    chassis_move_init->Motor_Wheel[0].Gearbox_Rate = 15.17f;

    Motor_C620_Init(&chassis_move_init->Motor_Wheel[1], &hfdcan2, CAN_Motor_ID_0x201, Motor_DJI_Control_Method_CURRENT,
                    Motor_DJI_Power_Limit_Status_DISABLE, 20.0f);
    chassis_move_init->Motor_Wheel[1].Gearbox_Rate = 15.17f;

    //跟随航向角偏移
    chassis_move_init->follow_yaw_offset = 2.34f;
    chassis_move_init->Target_Leg_l = 0.0f;
    chassis_move_init->Target_Leg_r = 0.0f;

    // 底盘 PID 初始化
    PID_Init(&chassis_move_init->PID_legL_Position, 1000.0f, 100.0f, 0.0f, 0.0f, 30.0f, 200.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_legR_Position, 1000.0f, 100.0f, 0.0f, 0.0f, 30.0f, 200.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_legL_Velocity, 50.0f, 0.0f, 0.0f, 0.0f, 5.0f, 30.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_legR_Velocity, 50.0f, 0.0f, 0.0f, 0.0f, 5.0f, 30.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_follow_yaw, 10.0f, 0.1f, 0.01f, 0.00f, 0.3f, 4.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
    PID_Init(&chassis_move_init->PID_roll, 80.0f, 0.0f, 10.0f, 0.0f, 0.0f, 90.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_tp, 50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.01f, 0.0f, 0.0f, 0.0f,0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_tp_omega, 1.5f, 0.0f, 0.0f, 0.00f, 0.0f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&chassis_move_init->PID_buffer, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.1f, 0.0f, 0.0f, 0.0f,0.0f, PID_D_First_DISABLE);

    chassis_move_init->PID_legL_Position.Target = 0.20f;
    chassis_move_init->PID_legR_Position.Target = 0.20f;
    chassis_move_init->PID_legL_Velocity.Target = 0.0f;
    chassis_move_init->PID_legR_Velocity.Target = 0.0f;
    //斜坡函数初始化
    slope_init(&chassis_move_init->Slope_X,12.0f / 1000.0f, 1.0f / 150.0f,Slope_First_REAL);

    Motor_DM_Normal_CAN_Send_Enable(&chassis_move_init->Motor_Joint[0]);
    Motor_DM_Normal_CAN_Send_Enable(&chassis_move_init->Motor_Joint[1]);
    Motor_DM_Normal_CAN_Send_Enable(&chassis_move_init->Motor_Joint[2]);
    Motor_DM_Normal_CAN_Send_Enable(&chassis_move_init->Motor_Joint[3]);

    //底盘卡尔曼滤波初始化
    chassis_kalman_init(&chassis_kalman);
    //状态机初始化
    FSM_Init(&chassis_move_init->FSM, 3, NORMAL);

}

//反馈更新
void Chassis_Feedback_Update(chassis_move_t *chassis)
{
    chassis->left_leg.phi1 = PI + normalizeAngleToPi_Robust(chassis->Motor_Joint[1].Rx_Data.Now_Angle);
    chassis->left_leg.phi4 = normalizeAngleToPi_Robust(chassis->Motor_Joint[0].Rx_Data.Now_Angle);
    chassis->left_leg.d_phi1 = chassis->Motor_Joint[1].Rx_Data.Now_Omega;
    chassis->left_leg.d_phi4 = chassis->Motor_Joint[0].Rx_Data.Now_Omega;

    chassis->right_leg.phi1 = PI - normalizeAngleToPi_Robust(chassis->Motor_Joint[2].Rx_Data.Now_Angle);
    chassis->right_leg.phi4 = -normalizeAngleToPi_Robust(chassis->Motor_Joint[3].Rx_Data.Now_Angle);
    chassis->right_leg.d_phi1 = -chassis->Motor_Joint[2].Rx_Data.Now_Omega;
    chassis->right_leg.d_phi4 = -chassis->Motor_Joint[3].Rx_Data.Now_Omega;

    VMC_Calc_1(&chassis->left_leg, (float)chassis_time/1000.0f);
    VMC_Calc_1(&chassis->right_leg, (float)chassis_time/1000.0f);
    Fitting_K_Calc(Fitting_K, P, chassis->left_leg.L0, chassis->right_leg.L0);

    chassis->pitch = -chassis->chassis_INS_point->Pitch;
    chassis->d_pitch = -chassis->chassis_INS_point->Gyro[0];
    chassis->roll = -(chassis->chassis_INS_point->Roll - 0.019f);
    chassis->d_roll = -chassis->chassis_INS_point->Gyro[1];
    // chassis->pitch = 0.0f;
    // chassis->d_pitch = 0.0f;
    // chassis->roll = 0.0f;

    chassis->theta_err = -(chassis->left_leg.theta - chassis->right_leg.theta);
    chassis->d_theta_err = -(chassis->left_leg.d_theta - chassis->right_leg.d_theta);
    chassis->Omega_l = chassis->Motor_Wheel[0].Rx_Data.Now_Omega - chassis->d_pitch + chassis->left_leg.d_theta;
    chassis->Omega_r = chassis->Motor_Wheel[1].Rx_Data.Now_Omega - chassis->d_pitch + chassis->right_leg.d_theta;
    chassis->Speed_l = 0.06f * chassis->Omega_l + chassis->left_leg.L0 * chassis->left_leg.d_theta * arm_cos_f32(chassis->left_leg.theta) + chassis->left_leg.d_L0 * arm_sin_f32(chassis->left_leg.theta);
    chassis->Speed_r = 0.06f * chassis->Omega_r + chassis->right_leg.L0 * chassis->right_leg.d_theta * arm_cos_f32(chassis->right_leg.theta) + chassis->right_leg.d_L0 * arm_sin_f32(chassis->right_leg.theta);
    chassis->Average_Speed = -(chassis->Speed_l - chassis->Speed_r) / 2.0f;
    //更新卡尔曼
    chassis_kalman_update(&chassis_kalman);
    chassis->X_filter += chassis->Velocity_filter * (float)chassis_time/1000.0f;

    chassis->M = 180.0f;
    chassis_move.F_mc = chassis->Velocity_filter * chassis->chassis_INS_point->Gyro[2] * 28.0f;
    Math_Constrain(&chassis->F_mc, 0.0f, 100.0f);
    chassis->spring_force_l = 100.0f - 10.0f*(1.0f - (chassis->left_leg.L0-0.1f)/0.28f);
    chassis->spring_force_r = 100.0f - 10.0f*(1.0f - (chassis->right_leg.L0-0.1f)/0.28f);

    chassis->err[0] = chassis->X_filter - chassis->Target_X;
    chassis->err[1] = chassis->Velocity_filter - chassis->Slope_Velocity_X;
    chassis->err[2] = 0;
    chassis->err[3] = chassis->chassis_INS_point->Gyro[2] - chassis->Target_Omega;
    chassis->err[4] = Max_Output((chassis->left_leg.theta - chassis->Target_Theta), 0.5f);
    chassis->err[5] = chassis->left_leg.d_theta - 0;
    chassis->err[6] = Max_Output((chassis->right_leg.theta - chassis->Target_Theta), 0.5f);
    chassis->err[7] = chassis->right_leg.d_theta - 0;
    chassis->err[8] = Max_Output((chassis->pitch - 0), 1.0f);
    chassis->err[9] = chassis->d_pitch - 0;

}


/**
 * @brief 根据底盘模式执行计算控制
 * @param
 * @return
 */
static void chassis_control_loop(chassis_move_t *chassis)
{
    if (chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        if (FSM_Get_Now_Status(&chassis_move.FSM) == NORMAL)
        {
            //正常pid计算
            chassis_normal_pid_loop(chassis);
            //lqr计算
            chassis_lqr_calc_to_motor(chassis);
        }
        else if (FSM_Get_Now_Status(&chassis_move.FSM) == OVER_TURN)
        {
            chassis_over_turn_mod(chassis);
        }
        else
        {
            chassis_control_leg_pid_loop(chassis);       /* 收腿：腿长压最短 + 髋力矩 */
        }
    }
    else
    {
        PID_Clear_Out(&chassis->PID_legL_Position);
        PID_Clear_Out(&chassis->PID_legR_Position);
        PID_Clear_Out(&chassis->PID_legL_Velocity);
        PID_Clear_Out(&chassis->PID_legR_Velocity);
        PID_Clear_Out(&chassis->PID_follow_yaw);
        //PID_Set_Integral_Error(&chassis_move.PID_follow_yaw, 0.0f);
        PID_Clear_Out(&chassis->PID_tp);
        PID_Clear_Out(&chassis->PID_tp_omega);
        chassis->T_wl = 0.0f;
        chassis->T_wr = 0.0f;
        chassis->left_leg.Tp = 0.0f;
        chassis->right_leg.Tp = 0.0f;
        chassis->left_leg.F0 = 0.0f;
        chassis->right_leg.F0 = 0.0f;
        chassis->left_leg.torque_set[0] = 0.0f;
        chassis->left_leg.torque_set[1] = 0.0f;
        chassis->right_leg.torque_set[0] = 0.0f;
        chassis->right_leg.torque_set[1] = 0.0f;
    }
}

/**
 * @brief 根据底盘模式执行计算控制
 * @param
 * @return
 */
static void chassis_normal_pid_loop(chassis_move_t *chassis)
{
    //设定速度
    chassis->Slope_X.Target = chassis->Target_Velocity_X;
    chassis->Slope_X.Now_Real = chassis->Velocity_filter;
    slope_calc(&chassis->Slope_X);
    chassis->Slope_Velocity_X = chassis->Slope_X.Out;

    // 设置左腿长度目标
    chassis->PID_legL_Position.Target += chassis->Target_Leg_l;
    Math_Constrain(&chassis->PID_legL_Position.Target, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    chassis->PID_legL_Position.Now = chassis->left_leg.L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legL_Position);

    // 设置右腿长度目标
    chassis->PID_legR_Position.Target += chassis->Target_Leg_r;
    Math_Constrain(&chassis->PID_legR_Position.Target, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    chassis->PID_legR_Position.Now = chassis->right_leg.L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legR_Position);

    // 设置左腿速度目标
    chassis->PID_legL_Velocity.Target = 0.0f;
    chassis->PID_legL_Velocity.Now = chassis->left_leg.d_L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legL_Velocity);

    // 设置右腿速度目标
    chassis->PID_legR_Velocity.Target = 0.0f;
    chassis->PID_legR_Velocity.Now = chassis->right_leg.d_L0;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_legR_Velocity);

    // 云台跟随偏航：目标与反馈保持同一坐标系，误差折叠到 ±π 走最短路径
    float yaw_now = chassis->Motor_Yaw.Rx_Data.Now_Angle;          // 原始多圈角
    float yaw_err = normalizeAngleToPi_Robust(chassis->follow_yaw_offset - yaw_now);
    // 设置yaw目标
    chassis->PID_follow_yaw.Target = chassis->follow_yaw_offset;
    chassis->PID_follow_yaw.Now = chassis->follow_yaw_offset - yaw_err;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_follow_yaw);

    // 设置横滚目标
    // chassis->PID_roll.Target = chassis->Target_Roll;
    // Math_Constrain(&chassis->PID_roll.Target, -PI/10.0f, PI/10.0f);
    // chassis->PID_roll.Now = chassis->roll;
    // PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_roll);
    //横滚力矩
    float roll_torque = chassis->PID_roll.K_P * (0.0f - chassis->roll) + chassis->PID_roll.K_D * (0.0f - chassis->d_roll);
    chassis->roll_out = roll_torque / B_TRACK;
    Math_Constrain(&chassis->roll_out, -20.0f, 20.0f);

    // 设置 theta 误差目标
    chassis->PID_tp.Target = chassis->Target_Theta;
    Math_Constrain(&chassis->PID_tp.Target, -PI/6.0f, PI/6.0f);
    chassis->PID_tp.Now = chassis->theta_err;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_tp);

    //设置 d_theta 误差目标
    chassis->PID_tp_omega.Target = chassis->PID_tp.Out;
    chassis->PID_tp_omega.Now = chassis->d_theta_err;
    PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_tp_omega);
}
/**
 * @brief 计算T矩阵给电机
 * @param
 * @return
 */
static void chassis_lqr_calc_to_motor(chassis_move_t *chassis)
{
    for (int i = 0; i < 4; i++)
    {
        chassis->T[i] = chassis->err[0] * Fitting_K[i][0]
                        + Max_Output(chassis->err[1] * Fitting_K[i][1], 16)
                        - Max_Output(chassis->err[2] * Fitting_K[i][2], 15.0f)
                        + chassis->err[3] * Fitting_K[i][3]
                        + chassis->err[4] * Fitting_K[i][4]
                        + chassis->err[5] * Fitting_K[i][5] * 0.94f
                        + chassis->err[6] * Fitting_K[i][6]
                        + chassis->err[7] * Fitting_K[i][7] * 0.94f
                        + chassis->err[8] * Fitting_K[i][8]
                        + chassis->err[9] * Fitting_K[i][9];
    }
    chassis->T_wl = chassis->T[0];
    chassis->T_wr = chassis->T[1];
    // chassis->left_leg.Tp = (chassis->T[2] - chassis->PID_tp_omega.Out);
    // chassis->right_leg.Tp = chassis->T[3] + chassis->PID_tp_omega.Out;
    chassis->left_leg.Tp = chassis->T[2];
    chassis->right_leg.Tp = chassis->T[3];
    chassis->left_leg.F0 = (chassis->PID_legL_Position.Out + chassis->PID_legL_Velocity.Out - chassis->spring_force_l + chassis->M / arm_cos_f32(chassis->left_leg.theta) - chassis->roll_out*2.0f - chassis->F_mc + 23.0f);
    chassis->right_leg.F0 = (-chassis->PID_legR_Position.Out - chassis->PID_legR_Velocity.Out + chassis->spring_force_r - chassis->M / arm_cos_f32(chassis->right_leg.theta) - chassis->roll_out*2.0f - chassis->F_mc);//通过leg_convert得到髋关节每个电机应有的力矩
    VMC_Calc_2(&chassis->left_leg);
    VMC_Calc_2(&chassis->right_leg);
}
/**
 * @brief 输出到电机
 * @param
 * @return
 */
static void chassis_output_to_motor(chassis_move_t *chassis)
{
    static int mod = 0;
    mod++;
    if (mod == 100)
    {
        chassis->PID_buffer.Target = 50.0f;
        chassis->PID_buffer.Now = power_heat_data.buffer_energy;
        PID_TIM_Adjust_PeriodElapsedCallback(&chassis->PID_buffer);
        mod = 0;
    }
    chassis->Super_Cap_Tx.cap_set_power = robot_state.chassis_power_limit * 100 - (int)chassis->PID_buffer.Out * 100;

    if (robot_state.chassis_output == 0 )
    {
        chassis->Super_Cap_Tx.cap_flag = 0;
    }
    else
    {
        chassis->Super_Cap_Tx.cap_flag = 1;
    }

    if (chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis->Motor_Joint[3].Control_Torque = float_constrain(chassis->right_leg.torque_set[1] * 1.0f, -35.0f, 35.0f);
        chassis->Motor_Joint[2].Control_Torque = float_constrain(chassis->right_leg.torque_set[0] * 1.0f, -35.0f, 35.0f);
        chassis->Motor_Joint[1].Control_Torque = float_constrain(-chassis->left_leg.torque_set[1] * 1.0f, -35.0f, 35.0f);
        chassis->Motor_Joint[0].Control_Torque = float_constrain(-chassis->left_leg.torque_set[0] * 1.0f, -35.0f, 35.0f);

        chassis->Motor_Wheel[0].Target_Current = float_constrain(4.0f * chassis->T_wl, -15.0f, 15.0f);
        chassis->Motor_Wheel[1].Target_Current = float_constrain(-4.0f * chassis->T_wr, -15.0f, 15.0f);

        // chassis->Motor_Joint[3].Control_Torque = 0.0f;
        // chassis->Motor_Joint[2].Control_Torque = 0.0f;
        // chassis->Motor_Joint[1].Control_Torque = 0.0f;
        // chassis->Motor_Joint[0].Control_Torque = 0.0f;
        //
        // chassis->Motor_Wheel[0].Target_Current = 0.0f;
        // chassis->Motor_Wheel[1].Target_Current = 0.0f;
    }
    else if (chassis_mode == CHASSIS_CHECK_IN)
    {
        chassis->Motor_Joint[3].Control_Torque = 0.0f;
        chassis->Motor_Joint[2].Control_Torque = 0.0f;
        chassis->Motor_Joint[1].Control_Torque = 0.0f;
        chassis->Motor_Joint[0].Control_Torque = 0.0f;

        // chassis->Motor_Wheel[0].Target_Current = float_constrain(2.0f * chassis->Target_Velocity_X, -3.0f, 3.0f);
        // chassis->Motor_Wheel[1].Target_Current = float_constrain(-2.0f * chassis->Target_Velocity_X, -3.0f, 3.0f);

        chassis->Motor_Wheel[0].Target_Current = 0.0f;
        chassis->Motor_Wheel[1].Target_Current = 0.0f;
    }
    // if (Can_Comm.Can_Control_Data.Rx_Data.robot_control_status==No_Control || chassis_mode == CHASSIS_ZERO_FORCE)
    if (chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis->Motor_Joint[3].Control_Torque = 0.0f;
        chassis->Motor_Joint[2].Control_Torque = 0.0f;
        chassis->Motor_Joint[1].Control_Torque = 0.0f;
        chassis->Motor_Joint[0].Control_Torque = 0.0f;

        chassis->Motor_Wheel[0].Target_Current = 0.0f;
        chassis->Motor_Wheel[1].Target_Current = 0.0f;
    }

    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&chassis->Motor_Joint[0]);
    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&chassis->Motor_Joint[1]);
    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&chassis->Motor_Joint[2]);
    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&chassis->Motor_Joint[3]);
    Motor_C620_TIM_Calculate_PeriodElapsedCallback(&chassis->Motor_Wheel[0]);
    Motor_C620_TIM_Calculate_PeriodElapsedCallback(&chassis->Motor_Wheel[1]);

    // ★ 与香橙派一致：位置通道按拍归零（位置项不介入，只保留速度环）
    // chassis->X_filter = 0.0f;
    // chassis->Target_X = chassis->X_filter;
}

void chassis_motor_keep_alive(chassis_move_t *chassis)
{
    static uint32_t Counter_KeepAlive = 0;

    if (Counter_KeepAlive++ > 100)
    {
        Counter_KeepAlive=0;
        //髋关节电机
        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&chassis->Motor_Joint[0]);
        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&chassis->Motor_Joint[1]);
        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&chassis->Motor_Joint[2]);
        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&chassis->Motor_Joint[3]);

        //轮电机
        Motor_C620_TIM_Alive_PeriodElapsedCallback(&chassis->Motor_Wheel[0]);
        Motor_C620_TIM_Alive_PeriodElapsedCallback(&chassis->Motor_Wheel[1]);
    }
}

void chassis_joint_motor_status_influence_output(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        if (chassis->Motor_Joint[i].Motor_DM_Status == Motor_DM_Status_DISABLE)
        {
            chassis->joint_motor_status[i] = Motor_Status_OFFLINE;
        }
        else if (chassis->Motor_Joint[i].Motor_DM_Status == Motor_DM_Status_ENABLE)
        {
            chassis->joint_motor_status[i] = Motor_Status_NORMAL;
        }
        else
        {
            chassis->joint_motor_status[i] = Motor_Status_OFFLINE;
        }
    }

}

void chassis_wheel_motor_status_influence_output(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        if (chassis->Motor_Wheel[i].CAN_Motor_Status == CAN_Motor_Status_DISABLE)
        {
            chassis->wheel_motor_status[i] = Motor_Status_OFFLINE;
        }
        else if (chassis->Motor_Wheel[i].CAN_Motor_Status == CAN_Motor_Status_ENABLE)
        {
            chassis->wheel_motor_status[i] = Motor_Status_NORMAL;
        }
        else
        {
            chassis->wheel_motor_status[i] = Motor_Status_OFFLINE;
        }
    }

}

void Chassis_Motor_Status_PeriodElapsedCallback(chassis_move_t *chassis)
{
    //电机在线状态检测
    chassis_motor_keep_alive(&chassis_move);

    //电机状态裁决
    chassis_joint_motor_status_influence_output(chassis);
    chassis_wheel_motor_status_influence_output(chassis);
}

static float Max_Output(float num,float max)
{
    if(num>=max) return max;
    else if(num<=-max) return -max;
    else return num;
}

/**
 * @brief  停车判定 + 静止位置闭环 + 静止腿角偏置
 * @note   必须在 Chassis_Feedback_Update() 之前调用，
 *         因为 err[0] / err[4] / err[6] 都在那里一次算完。
 *         对应香橙派 Chassis.cpp::Follow_Gimbal_Control() 里的 stop_flag 段（383-413 行）。
 */
void chassis_stop_state_machine(chassis_move_t *chassis)
{
    /* ---- 只在跟云台模式下生效；其它模式保持位置环关闭 ---- */
    if (chassis_mode != CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis->X_filter       = 0.0f;
        chassis->Target_X       = 0.0f;
        chassis->last_Target_X  = 0.0f;
        chassis->Target_Theta   = 0.0f;
        chassis->stop_flag      = 1;
        chassis->last_stop_flag = 1;
        return;
    }

    /* ================= 1. 停车判定（带回差，防止阈值附近抖动） ================= */
    if (Float_Math_Abs(chassis->Target_Velocity_X) < 0.05f &&
        Float_Math_Abs(chassis->Velocity_filter)    < 0.5f)
    {
        chassis->stop_flag = 1;      /* 指令小 且 实际也慢 -> 认为停稳 */
    }
    else if (Float_Math_Abs(chassis->Target_Velocity_X) > 0.01f)
    {
        chassis->stop_flag = 0;      /* 一有明确指令 -> 立即解除，保证起步不拖 */
    }

    /* ============ 2. 按状态分配 X_filter / Target_X / Target_Theta ============ */
    if (chassis->stop_flag)
    {
        if (!chassis->last_stop_flag)
        {
            /* 刚进入停止：把当前位置定为原点，这一拍还不启用位置环 */
            chassis->X_filter      = 0.0f;
            chassis->Target_X      = chassis->X_filter;
            chassis->last_Target_X = chassis->X_filter;
            chassis->Target_Theta  = 0.0f;
        }
        else
        {
            /* 持续停止：X_filter 不再清零，让它自由积分，
               于是 err[0] = X_filter - Target_X(0) 就是"回到停车点"的位置误差 */
            chassis->Target_X     = chassis->last_Target_X;   /* 恒为 0 */
            chassis->Target_Theta = 0.05f;              /* 静止预紧偏置 */
        }
    }
    else
    {
        /* 运动中：位置环彻底关掉；Target_Theta 归零，
           否则这个常值偏置会在行驶时吃掉绝大部分轮电流预算 */
        chassis->X_filter      = 0.0f;
        chassis->Target_X      = chassis->X_filter;
        chassis->last_Target_X = chassis->X_filter;
        chassis->Target_Theta  = 0.0f;
    }

    chassis->last_stop_flag = chassis->stop_flag;
}