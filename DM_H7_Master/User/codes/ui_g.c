//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_2_frame_t ui_g_group1_0;

ui_interface_line_t *ui_g_group1_chassisL = (ui_interface_line_t*)&(ui_g_group1_0.data[0]);
ui_interface_line_t *ui_g_group1_chassisR = (ui_interface_line_t*)&(ui_g_group1_0.data[1]);

void _ui_init_g_group1_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_group1_0.data[i].figure_name[0] = 0;
        ui_g_group1_0.data[i].figure_name[1] = 0;
        ui_g_group1_0.data[i].figure_name[2] = i + 0;
        ui_g_group1_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_group1_0.data[i].operate_type = 0;
    }

    ui_g_group1_chassisL->figure_type = 0;
    ui_g_group1_chassisL->operate_type = 1;
    ui_g_group1_chassisL->layer = 0;
    ui_g_group1_chassisL->color = 0;
    ui_g_group1_chassisL->start_x = 400;
    ui_g_group1_chassisL->start_y = 20;
    ui_g_group1_chassisL->width = 5;
    ui_g_group1_chassisL->end_x = 700;
    ui_g_group1_chassisL->end_y = 350;

    ui_g_group1_chassisR->figure_type = 0;
    ui_g_group1_chassisR->operate_type = 1;
    ui_g_group1_chassisR->layer = 0;
    ui_g_group1_chassisR->color = 0;
    ui_g_group1_chassisR->start_x = 1500;
    ui_g_group1_chassisR->start_y = 20;
    ui_g_group1_chassisR->width = 5;
    ui_g_group1_chassisR->end_x = 1200;
    ui_g_group1_chassisR->end_y = 350;


    ui_proc_2_frame(&ui_g_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group1_0, sizeof(ui_g_group1_0));
}

void _ui_update_g_group1_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_group1_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group1_0, sizeof(ui_g_group1_0));
}

void _ui_remove_g_group1_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_group1_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group1_0, sizeof(ui_g_group1_0));
}


void ui_init_g_group1() {
    _ui_init_g_group1_0();
}

void ui_update_g_group1() {
    _ui_update_g_group1_0();
}

void ui_remove_g_group1() {
    _ui_remove_g_group1_0();
}


ui_string_frame_t ui_g_group2_0;
ui_interface_string_t* ui_g_group2_NewText = &(ui_g_group2_0.option);

void _ui_init_g_group2_0() {
    ui_g_group2_0.option.figure_name[0] = 0;
    ui_g_group2_0.option.figure_name[1] = 1;
    ui_g_group2_0.option.figure_name[2] = 0;
    ui_g_group2_0.option.operate_type = 1;

    ui_g_group2_NewText->figure_type = 7;
    ui_g_group2_NewText->operate_type = 1;
    ui_g_group2_NewText->layer = 1;
    ui_g_group2_NewText->color = 0;
    ui_g_group2_NewText->start_x = 86;
    ui_g_group2_NewText->start_y = 763;
    ui_g_group2_NewText->width = 4;
    ui_g_group2_NewText->font_size = 40;
    ui_g_group2_NewText->str_length = 13;
    strcpy(ui_g_group2_NewText->string, "SEE YOU AGAIN");


    ui_proc_string_frame(&ui_g_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group2_0, sizeof(ui_g_group2_0));
}

void _ui_update_g_group2_0() {
    ui_g_group2_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group2_0, sizeof(ui_g_group2_0));
}

void _ui_remove_g_group2_0() {
    ui_g_group2_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group2_0, sizeof(ui_g_group2_0));
}

void ui_init_g_group2() {
    _ui_init_g_group2_0();
}

void ui_update_g_group2() {
    _ui_update_g_group2_0();
}

void ui_remove_g_group2() {
    _ui_remove_g_group2_0();
}

