#ifndef CTRLBOARD_H7_IMU_CHASSIS_BEHAVIOUR_H
#define CTRLBOARD_H7_IMU_CHASSIS_BEHAVIOUR_H
#include "ChassisL_Task.h"

void chassis_mode_set(chassis_move_t *chassis);
void chassis_set_control(chassis_move_t *chassis);
void chassis_over_turn_mod(chassis_move_t *chassis);
#endif //CTRLBOARD_H7_IMU_CHASSIS_BEHAVIOUR_H