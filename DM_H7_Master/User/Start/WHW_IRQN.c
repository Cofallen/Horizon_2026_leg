/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 15:42:35
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-07 11:27:00
 */

#include "WHW_IRQN.h"
#include "All_Init.h"
#include "chassisL.h"
#include "chassisR.h"
#include "vmc.h"
#include "get_K.h"
#include "BM_motor.h"
#include "observe.h"
#include "board2board.h"

uint8_t move;
static uint8_t TX[12] = {0x3A,0x98,0xfd,0x90,0x86,0xa7,0xff,0xf1,0xfd,0x90,0x86,0xa7};

//画UI与平时测试
void StartRobotUITask(void const *argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();
	
    for(;;)
    {
		
        vTaskDelayUntil(&currentTimeRobotUI, 1);
    }
}

//陀螺仪
void StartIMUTask(void const *argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();

    for(;;)
    {
        INS_Task(&IMU_Data, &temppid);
        vTaskDelayUntil(&currentTimeIMU, 1);
    }
}

//云台
void StartGimbalTask(void const *argument)
{
    // BM_EnableDisable(&hfdcan1, 0x01);
    // BM_set_ID(&hfdcan1, 2, 1);
    osDelay(100);
    BM_EnableDisable(&hfdcan2, 0x02);
    // BM_save_flash(&hfdcan1);
    osDelay(10);
    ChassisL_Init(&ALL_MOTOR, &Leg_l);
    ChassisR_Init(&ALL_MOTOR, &Leg_r);
    Vmc_Init(&Leg_l, 0.18);
    Vmc_Init(&Leg_r, 0.18);
    while (IMU_Data.pitch == 0.0f)
    {
        osDelay(1);
    }
    osDelay(1000); // 等待IMU数据稳定
    xvEstimateKF_Init(&vaEstimateKF, 0.001f);
    for(;;)
    {
        RUI_V_CONTAL.DWT_TIME.Move_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.Move_DWT_Count);
        Vmc_calcL(&Leg_l, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Vmc_calcR(&Leg_r, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisL_UpdateState(&Leg_l, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisR_UpdateState(&Leg_r, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Chassis_UpdateStateS(&Leg_l, &Leg_r, &ALL_MOTOR, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Chassis_GetStatus(&Leg_l, &Leg_r);
        Chassis_StateHandle(&Leg_l, &Leg_r);
        Chassis_Jump(&Leg_l, &Leg_r, &WHW_V_DBUS);
        ChassisL_Control(&Leg_l, &WHW_V_DBUS, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisR_Control(&Leg_r, &WHW_V_DBUS, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Chassis_GetTorque(&ALL_MOTOR, &Leg_l, &Leg_r, &WHW_V_DBUS);
        osDelay(1);
    }
}

//监控
void StartMonitorTask(void const * argument)
{
    portTickType currentTimeMonitor;
    currentTimeMonitor = xTaskGetTickCount();

    for(;;)
    {
        // BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
        // BM_Send_torque(&hfdcan2, 0x032, 0, 
        //                     Leg_r.torque_send.T1,
        //                     0,
        //                     Leg_r.torque_send.T2);
        BM_Send_torque(&hfdcan2, 0x032, Leg_l.torque_send.T1, 
                            Leg_r.torque_send.T1,
                            Leg_l.torque_send.T2,
                            Leg_r.torque_send.T2);
        osDelay(1);
        // DJI_Torque_Control(&hfdcan1, 0x200, 0.0f, 0.0f, 0, 0);

        DJI_Torque_Control(&hfdcan1, 0x200, Leg_r.torque_send.Tw, 0.0f, Leg_l.torque_send.Tw, 0);

        // DJI_Torque_Control(&hfdcan1, 0x200, Leg_l.torque_send.Tw, 0.0f, Leg_r.torque_send.Tw, 0.0f);
        osDelay(1);
    }
}

//系统辨识
void StartK3debugTask(void const * argument)
{
    portTickType currentTimeK3debug;
    currentTimeK3debug = xTaskGetTickCount();

	// k3debug_init(&ALL_MOTOR);
	DM_test_init();
	
    for(;;)
    {
        WHW_V_DBUS.ONLINE_JUDGE_TIME++;
        if (WHW_V_DBUS.ONLINE_JUDGE_TIME >= 300)
        {
            memset(&WHW_V_DBUS, 0, sizeof(WHW_V_DBUS));
            WHW_V_DBUS.Remote.S2_u8 = 1;
        }
        
//		k3debug_task(&ALL_MOTOR, &WHW_V_DBUS);
//        DM_test(&IMU_Data);
        VOFA_justfloat(ALL_MOTOR.left_front.DATA.voltage,
                       ALL_MOTOR.left_back.DATA.voltage,
                         ALL_MOTOR.right_front.DATA.voltage,
                        ALL_MOTOR.right_back.DATA.voltage,
                        RUI_V_CONTAL.DWT_TIME.Move_Dtime,
                        (float)Leg_l.status.stand,
                        (float)Leg_l.status.stand_count,(float)Leg_l.status.offGround,
                        Leg_l.LQR.Fn,Leg_r.LQR.Fn );
        // canx_send_data(&hfdcan3, 0x200, )
        // VOFA_justfloat((float)Leg_l.status.stand,
        //                 (float)Leg_l.stateSpace.theta,
        //                 (float)Leg_r.status.stand,
        //                 (float)Leg_r.stateSpace.theta,
        //                 Leg_l.LQR.Fn,
        //                 Leg_l.LQR.Fn,0,0,0,0);
        // VOFA_justfloat(Leg_l.pid.F0_l.error[0],
        //                 Leg_l.pid.F0_l.out,
        //                 Leg_l.pid.F0_l.Pout,
        //                 Leg_l.pid.F0_l.Iout,
        //                 Leg_l.pid.F0_l.Dout,
        //                 Leg_r.pid.F0_l.error[0],
        //                 Leg_r.pid.F0_l.out,
        //                 Leg_r.pid.F0_l.Pout,
        //                 Leg_r.pid.F0_l.Iout,
        //                 Leg_r.pid.F0_l.Dout);   
        Board_to_board_send(&boardTxData, WHW_V_DBUS.Remote.CH2_int16, 
                                          WHW_V_DBUS.Remote.CH3_int16,
                                          WHW_V_DBUS.Remote.S1_u8,
                                          WHW_V_DBUS.Remote.S2_u8);
        osDelay(2);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader1;
    uint8_t g_Can1RxData[64];

    FDCAN_RxHeaderTypeDef RxHeader3;
    uint8_t g_Can3RxData[64];
	
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
            switch(RxHeader1.Identifier)
            {
                case LEG_WL:
                    RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.left_wheel.DATA, g_Can1RxData);
                    break;
                case LEG_WR:
                    RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.right_wheel.DATA, g_Can1RxData);
                    break;
            }
        }

        if(hfdcan->Instance == FDCAN3)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can3RxData, 0, sizeof(g_Can3RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
            switch(RxHeader3.Identifier)
            {
                case 0x10C:
                    Board_to_board_recv(&boardRxData, g_Can3RxData);
                    break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader2;
    uint8_t g_Can2RxData[64];
	
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if(hfdcan->Instance == FDCAN2)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can2RxData, 0, sizeof(g_Can2RxData));	//接收前先清空数组
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
            switch(RxHeader2.Identifier)
            {
                case 0x051:
                    BM_Parse_drive(&ALL_MOTOR.left_front.DATA, g_Can2RxData);
                break;
                case 0x052:
                    BM_Parse_drive(&ALL_MOTOR.right_front.DATA, g_Can2RxData);
                break;
                case 0x053:
                    BM_Parse_drive(&ALL_MOTOR.left_back.DATA, g_Can2RxData);
                break;
                case 0x054:
                    BM_Parse_drive(&ALL_MOTOR.right_back.DATA, g_Can2RxData);
                break;
            }
        }
	}
}

extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		if (Size <= 18)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 18); // 接收完毕后重启
			RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 18); // 接收完毕后重启
			RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS);
			// memset(DBUS_RX_DATA, 0, 18);							   
		}
	}
	
	if(huart->Instance == USART10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)RX, 40);//上位机串口
	}
	
	if(huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 510);//图传串口
	}

    if(huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)ALL_RX.Data, 510);//裁判系统串口

        uint8_t data_length_7;

        data_length_7  = 510 - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);//计算接收到的数据长度
        Read_Data_first(&ALL_RX , &User_data , data_length_7);//测试函数：待修改
        memset((uint8_t*)ALL_RX.Data,0,data_length_7);//清零接收缓冲区
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收发生错误后重启
		memset(DBUS_RX_DATA, 0, 18);							   // 清除接收缓存		
	}
	
	if(huart->Instance == USART10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)RX, 40);//上位机串口
		memset((uint8_t *)RX, 0, 20);
	}
	
	if(huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 510);//图传串口
	}

    if(huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)ALL_RX.Data, 510);//裁判系统串口
    }
}

void USER_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{

}
