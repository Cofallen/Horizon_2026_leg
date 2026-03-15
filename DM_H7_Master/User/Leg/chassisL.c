#include "chassisL.h"
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

// 均单环 P：磕台阶 S: 倒地自启
float PID_S_LF[3] = {4.0f, 0.0f, 0.0f};
float PID_P_LF[3] = {10.0f, 0.0f, 10.0f};
float PID_S_LB[3] = {4.0f, 0.0f, 0.0f};
float PID_P_LB[3] = {10.0f, 0.0f, 10.0f};

const float PID_Follow_param[3] = {0.1f, 0.0f, 0.0f};
pid_type_def pid_follow = {0};
float outppp = 0;

void ChassisL_Init(MOTOR_Typedef *motor, Leg_Typedef *object)
{
    // BM_EnableDisable()
    ALL_MOTOR.left_front.DATA.pos_init_rad = 1.95352459f;
    ALL_MOTOR.left_back.DATA.pos_init_rad  = -0.25751704f;   // 读取lr都应取负
    ALL_MOTOR.left_wheel.DATA.Angle_Init   = ALL_MOTOR.left_wheel.DATA.Angle_Infinite;
    PID_Init(&motor->left_front.PID_P, 10.0f, 0.1f, PID_P_LF,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&motor->left_front.PID_S, 3.0f, 0.1f, PID_S_LF,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&motor->left_back.PID_P, 10.0f, 0.1f, PID_P_LB,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&motor->left_back.PID_S, 3.0f, 0.1f, PID_S_LB,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_init(&pid_follow, 0, PID_Follow_param, 2.0f, 0);

}
float a;
void ChassisL_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 更新状态
    object->stateSpace.theta = (PI / 2.0f - object->vmc_calc.phi0[POS] + imu->pitch /  57.3f);
    object->stateSpace.dtheta = Discreteness_Diff(&object->Discreteness.Theta, object->stateSpace.theta, dt);
    object->stateSpace.phi = -imu->pitch / 57.3f;
    a = Discreteness_Diff(&object->Discreteness.Phi, object->stateSpace.phi, dt);
    // object->stateSpace.dphi = Discreteness_Diff(&object->Discreteness.Phi, object->stateSpace.phi, dt);
    object->stateSpace.dphi = -imu->gyro[0];   

    // VOFA_justfloat(-imu->gyro[0], imu->gyro[1], imu->gyro[2],
    //                 a,0,0,0,0,0,0);
    object->stateSpace.ddtheta = Discreteness_Diff(&object->Discreteness.dTheta, object->stateSpace.dtheta, dt);
}

float kl = 1.0f, kr = 1.0f;
int16_t kl_count = 0, kr_count = 0;
// 用于更新两腿之间的相对状态
void Chassis_UpdateStateS(Leg_Typedef *Leg_l, Leg_Typedef *Leg_r, MOTOR_Typedef *motor, float dt)
{
    float s = 0.0f, dot_s_b = 0.0f, dot_s = 0.0f;
    float theta_wl = 0.0f, theta_wr = 0.0f;
    float dtheta_wl = 0.0f, dtheta_wr = 0.0f;

    theta_wl = -(float)(motor->left_wheel.DATA.Angle_Infinite - motor->left_wheel.DATA.Angle_Init) / 8192.0f * 360.0f / 57.3f / REDUCTION_RATIO;
    theta_wr =  (float)(motor->right_wheel.DATA.Angle_Infinite - motor->right_wheel.DATA.Angle_Init) / 8192.0f * 360.0f / 57.3f / REDUCTION_RATIO;
    dtheta_wl = Discreteness_Diff(&Leg_l->Discreteness.Theta_w, theta_wl, dt);
    dtheta_wr = Discreteness_Diff(&Leg_r->Discreteness.Theta_w, theta_wr, dt);

    s = RADIUS_WHEEL * (theta_wl + theta_wr) / 2.0f;
    dot_s_b = RADIUS_WHEEL * (dtheta_wl + dtheta_wr) / 2.0f;

    dot_s = dot_s_b + 0.5f * (Leg_l->vmc_calc.L0[POS] * Leg_r->stateSpace.dtheta * arm_cos_f32(Leg_l->stateSpace.theta) + Leg_r->vmc_calc.L0[POS] * Leg_l->stateSpace.dtheta * arm_cos_f32(Leg_r->stateSpace.theta)) \
                      + 0.5f * (Leg_l->vmc_calc.L0[VEL] * arm_sin_f32(Leg_l->stateSpace.theta) + Leg_r->vmc_calc.L0[VEL] * arm_sin_f32(Leg_r->stateSpace.theta));
    Leg_l->stateSpace.raw_dot_s = dot_s;
    Leg_r->stateSpace.raw_dot_s = dot_s;
    
    Leg_l->stateSpace.dot_s = xvEstimateKF_Update(&vaEstimateKF, dot_s, -IMU_Data.accel[0]);
    Leg_r->stateSpace.dot_s = xvEstimateKF_Update(&vaEstimateKF, dot_s, -IMU_Data.accel[0]);;
    Leg_l->stateSpace.s     = Discreteness_Sum(&Leg_l->Discreteness.dS, Leg_l->stateSpace.dot_s, dt);
    Leg_r->stateSpace.s     = Discreteness_Sum(&Leg_r->Discreteness.dS, Leg_r->stateSpace.dot_s, dt);

    Leg_l->LQR.delta = Leg_r->stateSpace.theta - Leg_l->stateSpace.theta;
    Leg_r->LQR.delta = Leg_r->stateSpace.theta - Leg_l->stateSpace.theta;

    slip_Check(Leg_l, Leg_r);

    // VOFA_justfloat(RUI_V_CONTAL.DWT_TIME.Move_Dtime,
    //               dot_s, Leg_l->stateSpace.dot_s, Leg_r->stateSpace.dot_s,
    //               s,
    //               (float)kl,(float)kr,0,0,0); 
}


void ChassisL_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt)
{   
    // 目标值获取应加上滤波 重写一个函数
    Chassis_Get_target(object, dbus, imu, dt);

    VOFA_justfloat(boardRxData.dataNeaten.yaw_imu, 
                        IMU_Data.yaw,
                        (float)outppp,
                        0,0,0,0,0,0,0);

    object->LQR.T_w = (object->LQR.K[0] * (object->stateSpace.theta - object->target.theta) +
                     object->LQR.K[1] * (object->stateSpace.dtheta - object->target.dtheta) +
                     object->LQR.K[2] * (object->stateSpace.s - object->target.s) +
                     object->LQR.K[3] * (object->stateSpace.dot_s - object->target.dot_s) +
                     object->LQR.K[4] * (object->stateSpace.phi - object->target.phi) +
                     object->LQR.K[5] * (object->stateSpace.dphi - object->target.dphi));

    object->LQR.T_p = (object->LQR.K[6] * (object->stateSpace.theta - object->target.theta) +
                      object->LQR.K[7] * (object->stateSpace.dtheta - object->target.dtheta) +
                      object->LQR.K[8] * (object->stateSpace.s - object->target.s) +
                      object->LQR.K[9] * (object->stateSpace.dot_s - object->target.dot_s) +
                      object->LQR.K[10] * (object->stateSpace.phi - object->target.phi) +
                      object->LQR.K[11] * (object->stateSpace.dphi - object->target.dphi));

    PID_calc(&object->pid.F0_l_p, object->vmc_calc.L0[POS], object->target.l0);
    PID_calc(&object->pid.F0_l_s, object->vmc_calc.L0[VEL], object->pid.F0_l_p.out);
    // PID_Calculate(&object->pid.F0_l_x, object->vmc_calc.L0[POS], object->target.l0);
    object->LQR.dF_0 = object->pid.F0_l_s.out;

    PID_calc(&object->pid.Roll, imu->roll / 57.3f, object->target.roll);
    object->LQR.dF_roll = object->pid.Roll.out;

    PID_calc(&object->pid.Delta, object->LQR.delta, object->target.d2theta);
    object->LQR.dF_delta = object->pid.Delta.out;

    PID_calc(&object->pid.Yaw, imu->YawTotalAngle / 57.3f, object->target.yaw);
    object->LQR.dF_yaw = object->pid.Yaw.out;
    // object->LQR.F_0 = -(object->LQR.dF_0 - object->LQR.dF_roll);
    // if (object->status.jump == 3 || object->status.jump == 4)
    //   object->LQR.F_0 = object->LQR.dF_0 + object->LQR.dF_roll; 
    // else 
      object->LQR.F_0 = (MASS_BODY / 2.0f * 9.81f / arm_cos_f32(object->stateSpace.theta) + object->LQR.dF_0 + object->LQR.dF_roll);
    // object->LQR.F_0 = 0;
    // pid修正
    object->LQR.T_p = object->LQR.T_p + object->LQR.dF_delta;
    object->LQR.T_w = object->LQR.T_w + object->LQR.dF_yaw;

    object->LQR.torque_setT[0] = object->vmc_calc.JRM[0][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[0][1] * object->LQR.T_p;
    object->LQR.torque_setT[1] = object->vmc_calc.JRM[1][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[1][1] * object->LQR.T_p;
    object->LQR.torque_setW  = object->LQR.T_w * kl;

    // 限幅
    (object->LQR.torque_setT[0] > object->limit.T_max) ? (object->LQR.torque_setT[0] = object->limit.T_max) : (object->LQR.torque_setT[0] < -object->limit.T_max) ? (object->LQR.torque_setT[0] = -object->limit.T_max) : 0;
    (object->LQR.torque_setT[1] > object->limit.T_max) ? (object->LQR.torque_setT[1] = object->limit.T_max) : (object->LQR.torque_setT[1] < -object->limit.T_max) ? (object->LQR.torque_setT[1] = -object->limit.T_max) : 0;
    (object->LQR.torque_setW > object->limit.W_max) ? (object->LQR.torque_setW = object->limit.W_max) : (object->LQR.torque_setW < -object->limit.W_max) ? (object->LQR.torque_setW = -object->limit.W_max) : 0;
    // if (dbus->Remote.S2_u8 == 1)
    // {
    //   object->LQR.torque_setT[0] = 0.0f;
    //   object->LQR.torque_setT[1] = 0.0f;
    //   object->LQR.torque_setW = 0.0f;
    // }
    (object->LQR.torque_setT[0] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[0] < -MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = -MAX_TORQUE_LEG_T) : 0;
    (object->LQR.torque_setT[1] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[1] < -MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = -MAX_TORQUE_LEG_T) : 0;
    (object->LQR.torque_setW > MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = MAX_TORQUE_LEG_W) : (object->LQR.torque_setW < -MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = -MAX_TORQUE_LEG_W) : 0;
}


void Chassis_SendTorque()
{
    RUI_V_CONTAL.DWT_TIME.TIM7_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.TIM7_DWT_Count);
    static uint8_t temp = 1;
    uint8_t count = 0;
    if (temp == 1){
      // BM_Send_torque(&hfdcan2, 0x032, Leg_l.torque_send.T1, 
      //                                 Leg_r.torque_send.T1,
      //                                Leg_l.torque_send.T2,
      //                                Leg_r.torque_send.T2);
        BM_Send_torque(&hfdcan2, 0x032, 0, 
                                  Leg_r.torque_send.T1,
                                  0,
                                  Leg_r.torque_send.T2);
        DJI_Torque_Control(&hfdcan1, 0x200, 0.0f, 0.0f, Leg_r.torque_send.Tw, 0.0f);

      // DJI_Torque_Control(&hfdcan1, 0x200, Leg_l.torque_send.Tw, 0.0f, Leg_r.torque_send.Tw, 0.0f);
      temp = -temp;
    }
    else{
      mit_ctrl(&hfdcan1, 0x02, 0,0,0,0, Leg_r.torque_send.T1);
      mit_ctrl(&hfdcan1, 0x04, 0,0,0,0, Leg_r.torque_send.T2);
      // mit_ctrl(&hcan1, 0x02, 0,0,0,0, 0);
      // mit_ctrl(&hcan1, 0x04, 0,0,0,0, 0);
      temp = -temp;
    }
}

// 决定输出力矩，选择
#define RATIO 1.0f

void Chassis_GetTorque(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus)
{
  left->torque_send.T1  = -left->LQR.torque_setT[0] * RATIO;
  left->torque_send.T2  = -left->LQR.torque_setT[1] * RATIO;
  left->torque_send.Tw  =  left->LQR.torque_setW * RATIO;
  right->torque_send.T1 =  right->LQR.torque_setT[0] * RATIO;
  right->torque_send.T2 =  right->LQR.torque_setT[1] * RATIO;
  right->torque_send.Tw = -right->LQR.torque_setW * RATIO;

  if (dbus->Remote.S2_u8 == 1)    // 离线
  {
    left->torque_send.T1 = 0.0f;
    left->torque_send.T2 = 0.0f;
    left->torque_send.Tw = 0.0f;
    right->torque_send.T1 = 0.0f;
    right->torque_send.T2 = 0.0f;
    right->torque_send.Tw = 0.0f;
  }
  else if (left->status.stand == 1 || right->status.stand == 1 )   // 倒地TODO
  {
    left->torque_send.T1 = motor->left_front.PID_S.Output;
    left->torque_send.T2 = motor->left_back.PID_S.Output;
    left->torque_send.Tw = 0.0f;
    right->torque_send.T1 = motor->right_front.PID_S.Output;
    right->torque_send.T2 = motor->right_back.PID_S.Output;
    right->torque_send.Tw = 0.0f;
  }
  else if (left->status.stand == 2 || right->status.stand == 2 )   // 倒地TODO
  {
    // left->torque_send.T1 = 0.0f;
    left->torque_send.T2 = left->torque_send.T1 * 0.1f;
    // left->torque_send.Tw = 0.0f;
    // right->torque_send.T1 = 0.0f;
    right->torque_send.T2 = right->torque_send.T1 * 0.1f;
    // right->torque_send.Tw = 0.0f;
  }
  else if (Leg_l.status.step_flag || Leg_r.status.step_flag)   // 模拟磕台阶测试
  {
    left->torque_send.T1 = motor->left_front.PID_P.Output;
    left->torque_send.T2 = motor->left_back.PID_P.Output;
    left->torque_send.Tw = 0.0f;
    right->torque_send.T1 = motor->right_front.PID_P.Output;
    right->torque_send.T2 = motor->right_back.PID_P.Output;
    right->torque_send.Tw = 0.0f;
  }
}

uint16_t cca = 0, ccb = 0;

void Chassis_GetStatus(Leg_Typedef *left, Leg_Typedef *right)
{   
    const uint32_t STEP_KEEP = 500;
    static uint32_t step_count = 0;

    uint8_t trigger = (Leg_l.vmc_calc.L0[POS] >= 0.3f && Leg_l.stateSpace.theta > 0.3f)
                    || (Leg_r.vmc_calc.L0[POS] >= 0.3f && Leg_r.stateSpace.theta > 0.3f);

    if (!left->status.step_flag && trigger)
    {
        // 触发，进入保持状态并清零计数
        left->status.step_flag = 1;
        right->status.step_flag = 1;
        step_count = 0;
    }
    else if (left->status.step_flag)
    {
        // 已在保持状态，计数增长；当触发消失或达到阈值时退出保持
        step_count++;
        if (!trigger || step_count >= STEP_KEEP)
        {
            left->status.step_flag = 0;
            right->status.step_flag = 0;
        }
    }
    else
    {
        // 未触发且未处于保持，保持为0
        left->status.step_flag = 0;
        right->status.step_flag = 0;
    }
    
    // 离地状态
    if (fabs(left->LQR.Fn) <= 20.0f)
    {
      left->status.offGround = 1;
      // memcpy(left->LQR.K, ChassisL_LQR_K_fall, sizeof(float) * 12);
    } else {
      left->status.offGround = 0;
      // memcpy(left->LQR.K, ChassisL_LQR_K, sizeof(float) * 12);
    }
    if (fabs(right->LQR.Fn) <= 20.0f)
    {
      right->status.offGround = 1;
      // memcpy(right->LQR.K, ChassisR_LQR_K_fall, sizeof(float) * 12);
    } else {
      right->status.offGround = 0;
      // memcpy(right->LQR.K, ChassisR_LQR_K, sizeof(float) * 12);
    }
    // memcpy(left->LQR.K, ChassisL_LQR_K, sizeof(float) * 12);
    // memcpy(right->LQR.K, ChassisR_LQR_K, sizeof(float) * 12);
    // Chassis_Fit_K(ChassisL_LQR_K_coeffs, left->vmc_calc.L0[POS], left->LQR.K);
    // Chassis_Fit_K(ChassisR_LQR_K_coeffs, right->vmc_calc.L0[POS], right->LQR.K);

    // 倒地自启
    uint8_t is_fallen = (fabs(left->stateSpace.theta) >= 1.2f) || (fabs(right->stateSpace.theta) >= 1.2f);
    // uint8_t is_fallen = (left->vmc_calc.L0[POS] >= 0.2f || right->vmc_calc.L0[POS] >= 0.2f) && (fabs(left->stateSpace.theta) >= 1.2f) || (fabs(right->stateSpace.theta) >= 1.2f);
    uint8_t can_recover = (fabs(left->stateSpace.theta) < 1.3f) && (fabs(right->stateSpace.theta) < 1.3f) && ((left->stateSpace.theta > 0) && (right->stateSpace.theta > 0));
    // uint8_t can_recover = (fabs(left->stateSpace.theta) < 1.3f) && (fabs(right->stateSpace.theta) < 1.3f);
    // 使用 left->status.stand 作为整车的状态标志 (0:正常, 1:倒地, 2:恢复)
    // if (fabsf(IMU_Data.pitch) > 60.0f || fabsf(IMU_Data.roll) > 60.0f)
    // {
    //   is_fallen = 1;
    // }
    
    switch (left->status.stand)
    {
    case 0:   // 正常状态
      if (is_fallen)
      {
        left->status.stand = 1;
        right->status.stand = 1;
      }
      break;
    case 1:   // 倒地状态
      if (can_recover)
      {
        left->status.stand = 2;
        right->status.stand = 2;
      }
      // 否则保持倒地
      break;
    case 2:   // 恢复状态
      // if (is_fallen)  // 如果再次倒地，切回上一个状态
      // {
      //   left->status.stand = 1;
      //   right->status.stand = 1;
      // }
      // else
      // {
        if (fabsf(left->stateSpace.theta - 0.3f) < 0.1f && fabsf(right->stateSpace.theta - 0.3f) < 0.1f)
        {
          left->status.stand_count++;
          right->status.stand_count++;
        }
      
        if (left->status.stand_count >= 200 || right->status.stand_count >= 200)
        {
          left->status.stand_count = 0;
          right->status.stand_count = 0;
          left->status.stand = 0;
          right->status.stand = 0;
        }
      // }
      break;
    default:
      break;
    }
}

float limitl = 6.0f, limitw = 2.0f;

// 不同状态处理，用于切换K阵
void Chassis_StateHandle(Leg_Typedef *left, Leg_Typedef *right)
{
    int machine_state = left->status.stand;
    static uint16_t time = 0;   // 板凳起立时间

    Chassis_Fit_K(ChassisL_LQR_K_coeffs, left->vmc_calc.L0[POS], left->LQR.K);
    Chassis_Fit_K(ChassisR_LQR_K_coeffs, right->vmc_calc.L0[POS], right->LQR.K);
    // memcpy(left->LQR.K , ChassisL_LQR_K, sizeof(ChassisL_LQR_K));
    // memcpy(right->LQR.K, ChassisR_LQR_K, sizeof(ChassisR_LQR_K));

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


    if (machine_state == 1) // 倒地
    {
      Chassis_Rotate(&ALL_MOTOR);
    }
    else if (machine_state == 2) // 恢复
    {
      //  memcpy(left->LQR.K , ChassisL_LQR_K_stand, sizeof(ChassisL_LQR_K));
      // memcpy(right->LQR.K, ChassisR_LQR_K_stand, sizeof(ChassisR_LQR_K));
      
      // 1. 收腿
      left->target.l0 = MIN_LEG_LENGTH;
      right->target.l0 = MIN_LEG_LENGTH;
      // memcpy(left->LQR.K, ChassisL_LQR_K_stand, sizeof(float) * 12);
      // memcpy(right->LQR.K, ChassisR_LQR_K_stand, sizeof(float) * 12);
      if (fabsf(left->stateSpace.theta) <= 0.3f || fabsf(right->stateSpace.theta ) <= 0.3f)
      {
        // left->limit.T_max = 0.0f;
        // right->limit.T_max = 0.0f;
        // left->limit.W_max = 1.0f;
        // right->limit.W_max = 1.0f;
      }
      else
      {
        left->limit.T_max = limitl;
        right->limit.T_max = limitl;
        left->limit.W_max = limitw;
        right->limit.W_max = limitw;
      }
      time++;
      // 2. 轮电机摆动，髋关节失能
      if (time >= 1000)
      {
        time = 0;
      }
    }
    else // 正常
    {
      left->limit.W_max = MAX_TORQUE_LEG_W;
      right->limit.W_max = MAX_TORQUE_LEG_W;
      left->limit.T_max = MAX_TORQUE_LEG_T;
      right->limit.T_max = MAX_TORQUE_LEG_T;
    }
}

// 获取目标值，使用规划
// 先一定速度旋转一定时间，后过零点后采用位置控制
static void getPIDAim_Speed(MOTOR_Typedef *motor)
{
  motor->left_front.DATA.aim = -1.0f; // 设定0.2rad/s
  if (Leg_l.stateSpace.theta >= 0.0f && Leg_l.stateSpace.theta <= 1.2f)
  {
    motor->left_front.DATA.aim = 0.0; // 最终位置
    motor->left_front.PID_S.Output = 0.0f;
  }
  motor->left_back.DATA.aim = -1.0f; // 设定0.2rad/s
  if (Leg_l.stateSpace.theta >= 0.0f && Leg_l.stateSpace.theta <= 1.2f)
  {
    motor->left_back.DATA.aim = 0.0f; // 最终位置
    motor->left_back.PID_S.Output = 0.0f;
  }
  motor->right_front.DATA.aim = 1.0f; // 设定0.2rad/s
  if (Leg_r.stateSpace.theta >= 0.0f && Leg_r.stateSpace.theta <= 1.2f)
  {
    motor->right_front.DATA.aim = 0.0f; // 最终位置
    motor->right_front.PID_S.Output = 0.0f;
  }
  motor->right_back.DATA.aim = 1.0f; // 设定0.2rad/s
  if (Leg_r.stateSpace.theta >= 0.0f && Leg_r.stateSpace.theta <= 1.2f)
  {
    motor->right_back.DATA.aim = 0.0f; // 最终位置
    motor->right_back.PID_S.Output = 0.0f;
  }
  
}

// 获取位置规划目标值
void get_PIDAim_Pos(MOTOR_Typedef *motor, uint8_t mode)
{
  switch (mode)
  {
  case 0:   // 倒地自启 left: theta 0

    break;
  
  default:
    break;
  }
}

// 正常后清除目标值
static void ClearAim(MOTOR_Typedef *motor)
{
  motor->left_front.DATA.aim = 0.0f;
  motor->right_front.DATA.aim = 0.0f;
  motor->left_back.DATA.aim = 0.0f;
  motor->right_back.DATA.aim = 0.0f;
}


// 倒地后旋转腿，采用pid测试
void Chassis_Rotate(MOTOR_Typedef *motor)
{
  getPIDAim_Speed(motor);

  // PID_Calculate(&motor->left_front.PID_P, Leg_l.stateSpace.theta, motor->left_front.DATA.aim);
  // PID_Calculate(&motor->left_back.PID_P, Leg_l.stateSpace.theta, motor->left_back.DATA.aim);
  // PID_Calculate(&motor->right_front.PID_P, Leg_r.stateSpace.theta, motor->right_front.DATA.aim);
  // PID_Calculate(&motor->right_back.PID_P, Leg_r.stateSpace.theta, motor->right_back.DATA.aim);

  PID_Calculate(&motor->left_front.PID_S, motor->left_front.DATA.vel, motor->left_front.DATA.aim);
  PID_Calculate(&motor->left_back.PID_S, motor->left_back.DATA.vel, motor->left_back.DATA.aim);
  PID_Calculate(&motor->right_front.PID_S, motor->right_front.DATA.vel, motor->right_front.DATA.aim);
  PID_Calculate(&motor->right_back.PID_S, motor->right_back.DATA.vel, motor->right_back.DATA.aim);

}

// 速度规划，用于获取速度实际值
float Chassis_SpeedPlan(float speed_target, float a_max, float T, float dt, Leg_Typedef *left, Leg_Typedef *right)
{
  static float t = 0.0f;  // 用于总时间
  if (fabsf(speed_target - left->stateSpace.dot_s) < 0.01f && fabsf(speed_target - right->stateSpace.dot_s) < 0.01f)
  {
    t = 0.0f;
    return speed_target;
  }

  float speed_out = 0.0f, k = 0.0f;
  if (speed_target > 0)
  {
    t += dt;
    k = a_max / T;
    speed_out = -1.0f / 2.0f * k * k * t * t + a_max * t;
    return speed_out;
  }
  if (speed_target < 0)
  {
    t += dt;
    k = a_max / T;
    speed_out = 1.0f / 2.0f * k * k * t * t - a_max * t;
    return speed_out;
  }
}



// 压缩 蹬腿 收腿 伸腿
typedef enum {
  idle = 0,
  compact,
  flight,
  retract,
  extend
}JumpState_t;

uint8_t state = 0;

uint8_t s1[2] = {0};

void Chassis_Jump(Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus)
{
  // VOFA_justfloat((float)s1[0], (float)s1[1],
  //                (float)state,
  //                left->vmc_calc.L0[POS],
  //                right->vmc_calc.L0[POS],
  //               left->target.l0,
  //               right->target.l0,
  //               0,0,0);
  s1[0] = dbus->Remote.S1_u8;
  if (s1[0] == 2 && s1[1] == 3)
  {
    state = compact;
  }
  s1[1] = s1[0];
  if (s1[0] == 3)
  {
    state = idle;
  }
  
  switch (state)
  {
  case idle:
    break;
  
  case compact:
    left->target.l0 -= 0.0002f;
    right->target.l0 -= 0.0002f;
    if (left->vmc_calc.L0[POS] <= 0.16f && right->vmc_calc.L0[POS] <= 0.16f)
    {
      state = flight;
    }
    break;

  case flight:
    left->target.l0 += 0.0012f;
    right->target.l0 += 0.0012f;
    // left->target.l0 = 0.4f;
    // right->target.l0 = 0.4f;
    left->pid.F0_l_p.Kp = 4000.0f;
    right->pid.F0_l_p.Kp = 4000.0f;
    left->pid.F0_l_s.max_out = 200.0f;    // 160太大，欠压16
    right->pid.F0_l_s.max_out = 200.0f;
    // left->pid.F0_l.max_out = 80.0f;
    // right->pid.F0_l.max_out = 80.0f;
    if (left->vmc_calc.L0[POS] >= 0.36f && right->vmc_calc.L0[POS] >= 0.36f)
    {
      state = retract;
    }
    break;

  case retract:
    // left->target.l0 -= 0.0008f;
    // right->target.l0 -= 0.0008f;
    left->target.l0 = 0.10f;
    right->target.l0 = 0.10f;
    left->limit.W_max = 0.0f;
    right->limit.W_max = 0.0f;
    // left->pid.F0_l.max_out = 30.0f;
    // right->pid.F0_l.max_out = 30.0f;
    if (left->vmc_calc.L0[POS] <= 0.14f && right->vmc_calc.L0[POS] <= 0.14f)
    {
      state = idle;
      left->limit.W_max = 6.0f;
      right->limit.W_max = 6.0f;
      left->pid.F0_l_p.Kp = 5000.0f;
      right->pid.F0_l_p.Kp = 5000.0f;
      left->pid.F0_l_p.Kd = 30000.0f;
      right->pid.F0_l_p.Kd = 30000.0f;
      left->pid.F0_l_s.max_out = 80.0f;
      right->pid.F0_l_s.max_out = 80.0f;
    }
    break;

  case extend:
    // left->target.l0 += 0.0008f;
    // right->target.l0 += 0.0008f;
    left->target.l0 = 0.14f;
    right->target.l0 = 0.14f;      
    left->pid.F0_l_p.Kd = 60000.0f;
    right->pid.F0_l_p.Kd = 60000.0f;
    // if (left->vmc_calc.L0[POS] >= 0.16f && right->vmc_calc.L0[POS] >= 0.16f)
    // {
      state = idle;
      left->pid.F0_l_p.Kp = 6000.0f;
      right->pid.F0_l_p.Kp = 6000.0f;
      left->pid.F0_l_p.Kd = 20000.0f;
      right->pid.F0_l_p.Kd = 20000.0f;
      left->pid.F0_l_s.max_out = 80.0f;
      right->pid.F0_l_s.max_out = 80.0f;
    // }
    break;

  default:
    state = idle;
    break;
  }
  left->status.jump = state;
  right->status.jump = state;
}

// 磕台阶 只考虑后面电机即可，自动实现收腿了，可能再伸腿需要两个电机
// 轨迹 theta 0 -> -120 -> -30 起立
void Chassis_DownUp(Leg_Typedef *left, Leg_Typedef *right, MOTOR_Typedef *motor, DBUS_Typedef *dbus)
{
  if (left->status.step_flag || right->status.step_flag)    // 测试为双环控位置变化
  {
    // 更换方式：双环控位置变化
    float left_aim = 0.0f, right_aim = 0.0f;
    left_aim = motor->left_back.DATA.pos_rad;     // 初始化为当前值
    right_aim = motor->right_back.DATA.pos_rad;   // 只考虑一个电机就行


    // 收腿时最小腿长
    Leg_l.target.l0 = MIN_LEG_LENGTH;
    Leg_r.target.l0 = MIN_LEG_LENGTH;

    // // 改变pid参数
    // motor->left_back.PID_P.Kp = 1.0f;
    // motor->left_back.PID_P.Ki = 0.0f;
    // motor->left_back.PID_P.Kd = 0.0f;
    // motor->right_back.PID_P.Kp = 2.0f;
    // motor->left_back.PID_P.Ki = 0.0f;
    // motor->left_back.PID_P.Kd = 0.0f;
    
    // motor->left_back.PID_S.Kp = 1.0f;
    // motor->left_back.PID_S.Ki = 0.0f;
    // motor->left_back.PID_S.Kd = 0.0f;
    // motor->right_back.PID_S.Kp = 2.0f;
    // motor->left_back.PID_S.Ki = 0.0f;
    // motor->left_back.PID_S.Kd = 0.0f;

    // motor->left_back.PID_S.MaxOut = 2.0f;
    // motor->right_back.PID_S.MaxOut = 2.0f;

    motor->left_back.DATA.aim = 1.4f;
    motor->right_back.DATA.aim = 1.4f;
    if (fabsf(Leg_l.stateSpace.theta - 1.3f) < 0.1f )
    {
      // motor->left_back.DATA.aim = 0.4;
      // motor->right_back.DATA.aim = 0.4f;
      // if (fabsf(Leg_l.stateSpace.theta - 0.523598766f) < 0.1f)
      // {
        motor->left_back.DATA.aim = 0.0f;
        motor->right_back.DATA.aim = 0.0f;
      // }
    }

    // PID_Calculate(&motor->left_front.PID_P, Leg_l.stateSpace.theta, motor->left_back.DATA.aim);
    PID_Calculate(&motor->left_back.PID_P, Leg_l.stateSpace.theta, motor->left_back.DATA.aim);
    // PID_Calculate(&motor->right_front.PID_P, Leg_l.stateSpace.theta, motor->left_back.DATA.aim);  // 使用左 的theta
    PID_Calculate(&motor->right_back.PID_P, Leg_l.stateSpace.theta, motor->left_back.DATA.aim);

    // PID_Calculate(&motor->left_front.PID_S, motor->left_front.DATA.vel, motor->left_front.PID_P.Output);
    // PID_Calculate(&motor->left_back.PID_S, motor->left_back.DATA.vel, motor->left_back.PID_P.Output);
    // PID_Calculate(&motor->right_front.PID_S, motor->right_front.DATA.vel, -motor->right_front.PID_P.Output);
    // PID_Calculate(&motor->right_back.PID_S, motor->right_back.DATA.vel, -motor->right_back.PID_P.Output);
  }
  
}