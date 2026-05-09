#include "StateSelect.h"
#include "chassisL.h"
#include "chassisR.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "controller.h"
#include "All_Init.h"
#include "vmc.h"
#include "get_K.h"
#include "pid_temp.h"
#include "BM_Motor.h"
#include "observe.h"
#include "board2board.h"
#include "get_target.h"
#include "KNN.h"

// float w[] = {8.23521264, 8.69834965, 1.78259253, 0.16771652, 1.07922558, 0.03220763, -0.53748833, 0.42575712, -0.58027362};
// float b = -6.20173577;
// float mean[] = {-30.41808307, -12.34488136, 0.00315275, -0.00464040, 1.35854588, -2.00639961, -0.01038345, 0.92982620, 9.67937703};
// float std[] = {94.37885327, 19.92452379, 0.38786233, 1.16555753, 5.68133384, 239.56580740, 0.33913288, 0.14331179, 2.84528473};

void Chassis_GetStatus(Leg_Typedef *left, Leg_Typedef *right)
{   
    // 离地状态
    // left->status.offGround = ground_check(&Leg_l, &IMU_Data, w, b, mean, std);
    // right->status.offGround = ground_check(&Leg_r, &IMU_Data, w, b, mean, std);
    // left->status.offGround = ground_check(&Leg_l, &IMU_Data);
    // right->status.offGround = ground_check(&Leg_r, &IMU_Data);
    float theta_avg;
    float L0_avg;

    theta_avg =
        0.5f *
        (left->stateSpace.theta +
         right->stateSpace.theta);

    L0_avg =
        0.5f *
        (left->vmc_calc.L0[POS] +
         right->vmc_calc.L0[POS]);

    switch(left->status.robot_state)
    {
        case STATE_BALANCE:

            if(fabsf(left->stateSpace.theta) > 1.2f ||
               fabsf(right->stateSpace.theta) > 1.2f)
            {
                left->status.fallen_count++;

                if(left->status.fallen_count > 100)
                {
                    left->status.robot_state =
                        STATE_FALLEN;

                    right->status.robot_state =
                        STATE_FALLEN;

                    left->status.fallen_count = 0;
                }
            }
            else
            {
                left->status.fallen_count = 0;
            }

            if(L0_avg > 0.34f &&
               theta_avg > 0.8f)
            {
                left->status.step_count++;

                if(left->status.step_count > 300)
                {
                    left->status.robot_state =
                        STATE_STEP;

                    right->status.robot_state =
                        STATE_STEP;

                    left->status.step_count = 0;
                }
            }
            else
            {
                left->status.step_count = 0;
            }

        break;

        case STATE_FALLEN:

            if(fabsf(left->stateSpace.theta - 1.2f) < 0.1f &&
               fabsf(right->stateSpace.theta - 1.2f) < 0.1f)
            {
                left->status.rising_count++;

                if(left->status.rising_count > 200)
                {
                    left->status.robot_state =
                        STATE_RISING;

                    right->status.robot_state =
                        STATE_RISING;

                    left->status.rising_count = 0;
                }
            }
            else
            {
                left->status.rising_count = 0;
            }

        break;

        case STATE_RISING:

            // ---------- 起立失败 ----------
            if(fabsf(theta_avg) > 1.45f)
            {
                left->status.robot_state = STATE_FALLEN;
                right->status.robot_state = STATE_FALLEN;

                left->status.rising_count = 0;
                break;
            }

            // ---------- 起立完成 ----------
            if(fabsf(theta_avg) < 1.15f)
            {
                left->status.rising_count++;

                if(left->status.rising_count > 15)
                {
                    left->status.robot_state = STATE_TRANSITION;
                    right->status.robot_state = STATE_TRANSITION;

                    left->status.rising_count = 0;
                }
            }
            else
            {
                left->status.rising_count = 0;
            }

        break;

        case STATE_TRANSITION:  // 限幅防止速度过快，起立过度2

            left->status.transition_count++;

            if(left->status.transition_count > 300)
            {
                left->status.robot_state = STATE_BALANCE;
                right->status.robot_state = STATE_BALANCE;

                left->status.transition_count = 0;
            }

        break;

        case STATE_STEP:

            if(theta_avg > 1.1f)
            {
                left->status.rising_count++;

                if(left->status.rising_count > 200)
                {
                    left->status.robot_state =
                        STATE_RISING;

                    right->status.robot_state =
                        STATE_RISING;

                    left->status.rising_count = 0;
                }
            }
            else
            {
                left->status.rising_count = 0;
            }

        break;
    }
}

void Chassis_FallenControl(Leg_Typedef *left,
                           Leg_Typedef *right)
{

}


// 不同状态限幅+处理
void Chassis_StateHandle(Leg_Typedef *left, Leg_Typedef *right)
{
    // 离地检测
    if (left->status.offGround == 1)
    {
      left->LQR.K[0] = 0;
      left->LQR.K[1] = 0;
      left->LQR.K[2] = 0;
      left->LQR.K[3] = 0;
      left->LQR.K[4] = 0;
      left->LQR.K[5] = 0;
      left->LQR.K[8] = 0;
      left->LQR.K[9] = 0;
      left->LQR.K[10] = 0;
      left->LQR.K[11] = 0;
    }
    if (right->status.offGround == 1)
    {
      right->LQR.K[0] = 0;
      right->LQR.K[1] = 0;
      right->LQR.K[2] = 0;
      right->LQR.K[3] = 0;
      right->LQR.K[4] = 0;
      right->LQR.K[5] = 0;
      right->LQR.K[8] = 0;
      right->LQR.K[9] = 0;
      right->LQR.K[10] = 0;
      right->LQR.K[11] = 0;
    }

      // left->limit.W_max = MAX_TORQUE_LEG_W;
      // right->limit.W_max = MAX_TORQUE_LEG_W;
      // left->limit.T_max = MAX_TORQUE_LEG_T;
      // right->limit.T_max = MAX_TORQUE_LEG_T;


    switch(left->status.robot_state)
    {
        case STATE_BALANCE:

            left->limit.T_max = MAX_TORQUE_LEG_T;
            right->limit.T_max = MAX_TORQUE_LEG_T;

            left->limit.W_max = MAX_TORQUE_LEG_W;
            right->limit.W_max = MAX_TORQUE_LEG_W;

        break;

        case STATE_FALLEN:

            left->limit.T_max = 0.0f;
            right->limit.T_max = 0.0f;

            left->limit.W_max = 0.0f;
            right->limit.W_max = 0.0f;

        break;

        case STATE_RISING:

            left->limit.T_max = 8.0f;
            right->limit.T_max = 8.0f;

            left->limit.W_max = 0.0f;
            right->limit.W_max = 0.0f;

        break;
        
        case STATE_TRANSITION:

            left->limit.T_max = 20.0f;
            right->limit.T_max = 20.0f;

            left->limit.W_max = 3.0f;
            right->limit.W_max = 3.0f;

        break;

        case STATE_STEP:

            left->limit.T_max = 0.0f;
            right->limit.T_max = 0.0f;

        break;
    }
}


// 选择控制方案
void Chassis_ControlSelect(MOTOR_Typedef *motor,
                           Leg_Typedef *left,
                           Leg_Typedef *right,
                           DBUS_Typedef *dbus,
                           float dt)
{
    switch(left->status.robot_state)
    {
        case STATE_BALANCE:
            Chassis_Fit_K(ChassisL_LQR_K_coeffs, left->vmc_calc.L0[POS], left->LQR.K);
            Chassis_Fit_K(ChassisR_LQR_K_coeffs, right->vmc_calc.L0[POS], right->LQR.K);
            ChassisL_Control(left, dbus, &IMU_Data, dt);
            ChassisR_Control(right, dbus, &IMU_Data, dt);

        break;

        case STATE_FALLEN:

            Chassis_FallenControl(left, right);

        break;

        case STATE_RISING:

            // LQR直接限幅起立（你说的正确方案）
            Chassis_Fit_K(ChassisL_LQR_K_rising, left->vmc_calc.L0[POS], left->LQR.K);
            Chassis_Fit_K(ChassisR_LQR_K_rising, right->vmc_calc.L0[POS], right->LQR.K);
            left->target.l0  = MIN_LEG_LENGTH;
            right->target.l0 = MIN_LEG_LENGTH;
            ChassisL_Control(left, dbus, &IMU_Data, dt);
            ChassisR_Control(right, dbus, &IMU_Data, dt);

            // left->limit.T_max = 6.0f;
            // right->limit.T_max = 6.0f;

        break;

        case STATE_TRANSITION:

            Chassis_Fit_K(ChassisL_LQR_K_coeffs, left->vmc_calc.L0[POS], left->LQR.K);
            Chassis_Fit_K(ChassisR_LQR_K_coeffs, right->vmc_calc.L0[POS], right->LQR.K);
            // left->target.l0  = 0.15f;
            // right->target.l0 = 0.15f;
            ChassisL_Control(left, dbus, &IMU_Data, dt);
            ChassisR_Control(right, dbus, &IMU_Data, dt);

        break;

        case STATE_STEP:

            // Chassis_DownUp(left, right, motor, dbus);

        break;
    }
}