// /*
//  *                                                     __----~~~~~~~~~~~------___
//  *                                    .  .   ~~//====......          __--~ ~~
//  *                    -.            \_|//     |||\\  ~~~~~~::::... /~
//  *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
//  *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
//  *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
//  *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
//  *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
//  *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
//  *           '         ~-|      /|    |-~\~~       __--~~
//  *                       |-~~-_/ |    |   ~\_   _-~            /\
//  *                            /  \     \__   \/~                \__
//  *                        _--~ _/ | .-~~____--~-/                  ~~==.
//  *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
//  *                                  -_     ~\      ~~---l__i__i__i--~~_/
//  *                                  _-~-__   ~)  \--______________--~~
//  *                                //.-~~~-~_--~- |-------~~~~~~~~
//  *                                       //.-~~~--\
//  *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  * 
//  *                               神兽保佑            永无BUG
//  */
// #include "Monitor_Task.h"

// /************************************************************万能分隔符**************************************************************
//  * 	@author:			//小瑞
//  *	@performance:	    //ROOT函数
//  *	@parameter:		    //
//  *	@time:				//23-04-13 17:38
//  *	@ReadMe:			//
//  ************************************************************万能分隔符**************************************************************/
// void RUI_F_ROOT(RUI_ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, Data_RX_0x302_t *RX_0x302, MOTOR_Typedef *MOTOR)
// {
//     RUI_F_ROOT_ALL_STATUS(Root, DBUS, RX_0x302, MOTOR);
//     RUI_F_MASTER_LOCATION_JUDGE(Root);
//     RUI_F_LED_SHOW_STATUS(Root);
// }

// /************************************************************万能分隔符**************************************************************
// // * 	@author:			//WHW
// // *	@performance:	    //大疆电机离线检测
// // *	@parameter:		    //
// // *	@time:				//23-04-13 15:01
// // *	@ReadMe:			//
// // ************************************************************万能分隔符**************************************************************/
// uint8_t DJI_F_MOTOR_STATUS(DJI_MOTOR_DATA_Typedef* DATA)
// {
//     DATA->ONLINE_JUDGE_TIME--;

//     if (DATA->ONLINE_JUDGE_TIME < 5)
//     {
//         DATA->ONLINE_JUDGE_TIME = 0;
//         return RUI_DF_OFFLINE;
//     }
//     else
//         return RUI_DF_ONLINE;
// }

// /************************************************************万能分隔符**************************************************************
// // * 	@author:			//WHW
// // *	@performance:	    //达妙电机离线检测
// // *	@parameter:		    //
// // *	@time:				//23-04-13 15:01
// // *	@ReadMe:			//
// ************************************************************万能分隔符**************************************************************/
// uint8_t DM_F_MOTOR_STATUS(DM_MOTOR_DATA_Typdef* DATA)
// {
//   DATA->ONLINE_JUDGE_TIME--;

//   if (DATA->ONLINE_JUDGE_TIME < 5)
//   {
//     DATA->ONLINE_JUDGE_TIME = 0;
//     return RUI_DF_OFFLINE;
//   }
//   else
//     return RUI_DF_ONLINE;
// }

// /************************************************************万能分隔符**************************************************************
//  * 	@author:			//小瑞
//  *	@performance:	    //总的离线检测
//  *	@parameter:		    //
//  *	@time:				//23-03-29 19:34
//  *	@ReadMe:			//
//  *  @LastUpDataTime:    //2023-04-13 15:22    bestrui
//  *  @UpData：           //电机检测封装成函数
//  ************************************************************万能分隔符**************************************************************/
// void RUI_F_ROOT_ALL_STATUS(RUI_ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, Data_RX_0x302_t *RX_0x302, MOTOR_Typedef *MOTOR)
// {
//     //遥控在线监测
//     if (DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME < 5)
//     {
//         DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME = 3;
//         Root->RM_DBUS = RUI_DF_OFFLINE;
//     }
//     else
//     {
//         Root->RM_DBUS = RUI_DF_ONLINE;
//     }
//     DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME--;

//     //自定义控制器在线监测
//     if (RX_0x302->channal.ONLINE_JUDGE_TIME_0x302 < 5)
//     {
//         RX_0x302->channal.ONLINE_JUDGE_TIME_0x302 = 3;
//         Root->RM_RX = RUI_DF_OFFLINE;
//     }
//     else
//     {
//         Root->RM_RX = RUI_DF_ONLINE;
//     }
//     RX_0x302->channal.ONLINE_JUDGE_TIME_0x302--;

//     //电机在线检测
//     Root->MOTOR_HEAD_Pitch = DJI_F_MOTOR_STATUS(&MOTOR->DJI_2006_Pitch.DATA);
//     Root->MOTOR_HEAD_Roll = DJI_F_MOTOR_STATUS(&MOTOR->DJI_2006_Roll.DATA);

// }

// /************************************************************万能分隔符**************************************************************
//  * 	@author:			//小瑞
//  *	@performance:	    //主控位置判断
//  *	@parameter:		    //
//  *	@time:				//23-03-29 19:34
//  *	@ReadMe:			//
//  ************************************************************万能分隔符**************************************************************/
// uint8_t RUI_F_MASTER_LOCATION_JUDGE(RUI_ROOT_STATUS_Typedef *Root)
// {
//     Root->MASTER_LOCATION = RUI_DF_MASTER_LOCATION_HEAD;
//     return RUI_DF_READY;
// }

// /************************************************************万能分隔符**************************************************************
//  * 	@author:			//小瑞
//  *	@performance:	    //用LED灯显示状态
//  *	@parameter:		    //
//  *	@time:				//23-04-13 17:36
//  *	@ReadMe:			//
//  *  @LastUpDataTime:    //2023-05-07 17:03    bestrui
//  *  @UpData：           //底盘正常改为绿灯闪烁
//  ************************************************************万能分隔符**************************************************************/
// void RUI_F_LED_SHOW_STATUS(RUI_ROOT_STATUS_Typedef *Root)
// {
//     switch (Root->MASTER_LOCATION)
//     {
//         case RUI_DF_MASTER_LOCATION_JUDGEING:
//             RGB_Ctrl(RUI_DF_LED_YELLOW, 100.0f, 0xf0);
//         break;

//         case RUI_DF_MASTER_LOCATION_HEAD:
//         {
//             if (Root->RM_DBUS           == RUI_DF_ONLINE &&
//             Root->MOTOR_HEAD_Pitch      == RUI_DF_ONLINE &&
//             Root->MOTOR_HEAD_Roll       == RUI_DF_ONLINE)
//             {
//                 RGB_Ctrl(RUI_DF_LED_CYAN, 200.0f, 0xf0);
//             }
//             if (Root->RM_DBUS           == RUI_DF_OFFLINE &&
//             (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
//             Root->MOTOR_HEAD_Roll	== RUI_DF_OFFLINE))
//             {
//                 RGB_Ctrl(RUI_DF_LED_PURPLE, 50.0f, 0xf0);
//             }
//             if (Root->RM_DBUS           == RUI_DF_ONLINE &&
//             (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
//             Root->MOTOR_HEAD_Roll	== RUI_DF_OFFLINE))
//             {
//                 RGB_Ctrl(RUI_DF_LED_CYAN, 50.0f, 0xf0);
//             }
//         }
//         break;
//     }
// }
