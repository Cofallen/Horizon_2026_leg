#include "Monitor_Task.h"
#include "All_Init.h"
#include "ui_interface.h"
#include "cmsis_os.h"
#include "ui_g.h"

void RobotUI_Init(void)
{
   ui_self_id = User_data.robot_status.robot_id;
    ui_init_g_group1();
    osDelay(40);
    ui_init_g_group2();
    // ui_init_1_Ungroup();
    // ui_init_default_Ungroup();
    osDelay(40);
}


void RobotUI_Update(void)
{
    // ui_update_1_Ungroup();
    // osDelay(40);
    // ui_update_default_Ungroup();
    // osDelay(40);
}