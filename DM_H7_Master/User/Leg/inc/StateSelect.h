#ifndef __STATESELECT_H
#define __STATESELECT_H

#include "main.h"
#include "vmc.h"
#include "All_Init.h"

void Chassis_GetStatus(Leg_Typedef *left, Leg_Typedef *right);
void Chassis_FallenControl(Leg_Typedef *left,
                           Leg_Typedef *right);
void Chassis_StateHandle(Leg_Typedef *left, Leg_Typedef *right);
void Chassis_ControlSelect(MOTOR_Typedef *motor,
                           Leg_Typedef *left,
                           Leg_Typedef *right,
                           DBUS_Typedef *dbus,
                           float dt);
#endif