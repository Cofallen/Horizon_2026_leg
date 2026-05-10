#include "RecoveryControl.h"
#include "pid_temp.h"
#include "All_Init.h"
#include "legRotate.h"

static pid_type_def roll_pid;

static const float roll_param[3] =
{
    0.1f,
    0.0f,
    0.0f
};

void Recovery_Init(void)
{
    PID_init(&roll_pid,
             PID_POSITION,
             roll_param,
             0.4f,
             0.0f);
}

void RollRecovery_Control(MOTOR_Typedef *motor,
                          Leg_Typedef *left,
                          Leg_Typedef *right)
{
    float roll;

    float iq;

    float torque;

    float err_l;
    float err_r;

    roll =
        IMU_Data.roll / 57.3f;

    if(fabsf(roll) < 0.05f)
    {
        PID_clear(&roll_pid);

        return;
    }

    err_l =
        fabsf(
            LegRotate.theta_l -
            LegRotate.target_l_final 
        );

    err_r =
        fabsf(
            LegRotate.theta_r -
            LegRotate.target_r_final 
        );

    if(roll < 0.0f)
    {
        if(err_l > 0.40f)
        {
            PID_clear(&roll_pid);

            return;
        }

        iq =
            0.5f *
            (
                motor->left_front.DATA.IQ +
                motor->left_back.DATA.IQ
            );

        torque =
            -PID_calc(&roll_pid,
                     iq,
                     8.0f * roll);

        left->LQR.torque_setT[0] += torque;

        left->LQR.torque_setT[1] += torque;
    }
    else
    {
        if(err_r > 0.20f)
        {
            PID_clear(&roll_pid);

            return;
        }

        iq =
            0.5f *
            (
                motor->right_front.DATA.IQ +
                motor->right_back.DATA.IQ
            );

        torque =
            PID_calc(&roll_pid,
                     iq,
                     8.0f * roll);

        right->LQR.torque_setT[0] += torque;

        right->LQR.torque_setT[1] += torque;
    }
}