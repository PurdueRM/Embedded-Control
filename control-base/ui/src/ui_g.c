//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"
#include "ui_g.h"

ui_5_frame_t ui_g_0_0;

ui_interface_line_t *ui_g_0_Path_R = (ui_interface_line_t*)&(ui_g_0_0.data[0]);
ui_interface_round_t *ui_g_0_Aim = (ui_interface_round_t*)&(ui_g_0_0.data[1]);
ui_interface_line_t *ui_g_0_Path_L = (ui_interface_line_t*)&(ui_g_0_0.data[2]);

void _ui_init_g_0_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_0_0.data[i].figure_name[0] = 0;
        ui_g_0_0.data[i].figure_name[1] = 0;
        ui_g_0_0.data[i].figure_name[2] = i + 0;
        ui_g_0_0.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_0_0.data[i].operate_type = 0;
    }

    ui_g_0_Path_R->figure_type = 0;
    ui_g_0_Path_R->operate_type = 1;
    ui_g_0_Path_R->layer = 0;
    ui_g_0_Path_R->color = 2;
    ui_g_0_Path_R->start_x = 1450;
    ui_g_0_Path_R->start_y = 170;
    ui_g_0_Path_R->width = 2;
    ui_g_0_Path_R->end_x = 1280;
    ui_g_0_Path_R->end_y = 430;

    ui_g_0_Aim->figure_type = 2;
    ui_g_0_Aim->operate_type = 1;
    ui_g_0_Aim->layer = 0;
    ui_g_0_Aim->color = 2;
    ui_g_0_Aim->start_x = 936;
    ui_g_0_Aim->start_y = 492;
    ui_g_0_Aim->width = 5;
    ui_g_0_Aim->r = 15;

    ui_g_0_Path_L->figure_type = 0;
    ui_g_0_Path_L->operate_type = 1;
    ui_g_0_Path_L->layer = 0;
    ui_g_0_Path_L->color = 2;
    ui_g_0_Path_L->start_x = 480;
    ui_g_0_Path_L->start_y = 170;
    ui_g_0_Path_L->width = 2;
    ui_g_0_Path_L->end_x = 650;
    ui_g_0_Path_L->end_y = 430;


    ui_proc_5_frame(&ui_g_0_0);
    SEND_MESSAGE((uint8_t *) &ui_g_0_0, sizeof(ui_g_0_0));
}

void _ui_update_g_0_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_0_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_0_0);
    SEND_MESSAGE((uint8_t *) &ui_g_0_0, sizeof(ui_g_0_0));
}

void _ui_remove_g_0_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_0_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_0_0);
    SEND_MESSAGE((uint8_t *) &ui_g_0_0, sizeof(ui_g_0_0));
}

ui_string_frame_t ui_g_0_1;
ui_interface_string_t* ui_g_0_Flywheel_Text = &(ui_g_0_1.option);

void _ui_init_g_0_1() {
    ui_g_0_1.option.figure_name[0] = 0;
    ui_g_0_1.option.figure_name[1] = 0;
    ui_g_0_1.option.figure_name[2] = 3;
    ui_g_0_1.option.operate_type = 1;

    ui_g_0_Flywheel_Text->figure_type = 7;
    ui_g_0_Flywheel_Text->operate_type = 1;
    ui_g_0_Flywheel_Text->layer = 0;
    ui_g_0_Flywheel_Text->color = 1;
    ui_g_0_Flywheel_Text->start_x = 1500;
    ui_g_0_Flywheel_Text->start_y = 850;
    ui_g_0_Flywheel_Text->width = 3;
    ui_g_0_Flywheel_Text->font_size = 25;
    ui_g_0_Flywheel_Text->str_length = 8;
    strcpy(ui_g_0_Flywheel_Text->string, "FLYWHEEL");


    ui_proc_string_frame(&ui_g_0_1);
    SEND_MESSAGE((uint8_t *) &ui_g_0_1, sizeof(ui_g_0_1));
}

void _ui_update_g_0_1() {
    ui_g_0_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_0_1);
    SEND_MESSAGE((uint8_t *) &ui_g_0_1, sizeof(ui_g_0_1));
}

void _ui_remove_g_0_1() {
    ui_g_0_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_0_1);
    SEND_MESSAGE((uint8_t *) &ui_g_0_1, sizeof(ui_g_0_1));
}
ui_string_frame_t ui_g_0_2;
ui_interface_string_t* ui_g_0_Spintop_Text = &(ui_g_0_2.option);

void _ui_init_g_0_2() {
    ui_g_0_2.option.figure_name[0] = 0;
    ui_g_0_2.option.figure_name[1] = 0;
    ui_g_0_2.option.figure_name[2] = 4;
    ui_g_0_2.option.operate_type = 1;

    ui_g_0_Spintop_Text->figure_type = 7;
    ui_g_0_Spintop_Text->operate_type = 1;
    ui_g_0_Spintop_Text->layer = 0;
    ui_g_0_Spintop_Text->color = 1;
    ui_g_0_Spintop_Text->start_x = 1500;
    ui_g_0_Spintop_Text->start_y = 780;
    ui_g_0_Spintop_Text->width = 3;
    ui_g_0_Spintop_Text->font_size = 25;
    ui_g_0_Spintop_Text->str_length = 7;
    strcpy(ui_g_0_Spintop_Text->string, "SPINTOP");


    ui_proc_string_frame(&ui_g_0_2);
    SEND_MESSAGE((uint8_t *) &ui_g_0_2, sizeof(ui_g_0_2));
}

void _ui_update_g_0_2() {
    ui_g_0_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_0_2);
    SEND_MESSAGE((uint8_t *) &ui_g_0_2, sizeof(ui_g_0_2));
}

void _ui_remove_g_0_2() {
    ui_g_0_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_0_2);
    SEND_MESSAGE((uint8_t *) &ui_g_0_2, sizeof(ui_g_0_2));
}
ui_string_frame_t ui_g_0_3;
ui_interface_string_t* ui_g_0_Autoaim_Text = &(ui_g_0_3.option);

void _ui_init_g_0_3() {
    ui_g_0_3.option.figure_name[0] = 0;
    ui_g_0_3.option.figure_name[1] = 0;
    ui_g_0_3.option.figure_name[2] = 5;
    ui_g_0_3.option.operate_type = 1;

    ui_g_0_Autoaim_Text->figure_type = 7;
    ui_g_0_Autoaim_Text->operate_type = 1;
    ui_g_0_Autoaim_Text->layer = 0;
    ui_g_0_Autoaim_Text->color = 1;
    ui_g_0_Autoaim_Text->start_x = 1500;
    ui_g_0_Autoaim_Text->start_y = 710;
    ui_g_0_Autoaim_Text->width = 3;
    ui_g_0_Autoaim_Text->font_size = 25;
    ui_g_0_Autoaim_Text->str_length = 7;
    strcpy(ui_g_0_Autoaim_Text->string, "AUTOAIM");


    ui_proc_string_frame(&ui_g_0_3);
    SEND_MESSAGE((uint8_t *) &ui_g_0_3, sizeof(ui_g_0_3));
}

void _ui_update_g_0_3() {
    ui_g_0_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_0_3);
    SEND_MESSAGE((uint8_t *) &ui_g_0_3, sizeof(ui_g_0_3));
}

void _ui_remove_g_0_3() {
    ui_g_0_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_0_3);
    SEND_MESSAGE((uint8_t *) &ui_g_0_3, sizeof(ui_g_0_3));
}
ui_string_frame_t ui_g_0_4;
ui_interface_string_t* ui_g_0_Supercap_Text = &(ui_g_0_4.option);

void _ui_init_g_0_4() {
    ui_g_0_4.option.figure_name[0] = 0;
    ui_g_0_4.option.figure_name[1] = 0;
    ui_g_0_4.option.figure_name[2] = 6;
    ui_g_0_4.option.operate_type = 1;

    ui_g_0_Supercap_Text->figure_type = 7;
    ui_g_0_Supercap_Text->operate_type = 1;
    ui_g_0_Supercap_Text->layer = 0;
    ui_g_0_Supercap_Text->color = 6;
    ui_g_0_Supercap_Text->start_x = 420;
    ui_g_0_Supercap_Text->start_y = 820;
    ui_g_0_Supercap_Text->width = 3;
    ui_g_0_Supercap_Text->font_size = 25;
    ui_g_0_Supercap_Text->str_length = 5;
    strcpy(ui_g_0_Supercap_Text->string, "CAP: ");


    ui_proc_string_frame(&ui_g_0_4);
    SEND_MESSAGE((uint8_t *) &ui_g_0_4, sizeof(ui_g_0_4));
}

void _ui_update_g_0_4() {
    ui_g_0_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_0_4);
    SEND_MESSAGE((uint8_t *) &ui_g_0_4, sizeof(ui_g_0_4));
}

void _ui_remove_g_0_4() {
    ui_g_0_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_0_4);
    SEND_MESSAGE((uint8_t *) &ui_g_0_4, sizeof(ui_g_0_4));
}

void ui_init_g_0() {
    _ui_init_g_0_0();osDelay(SEND_INTERVAL_MS);
    _ui_init_g_0_1();osDelay(SEND_INTERVAL_MS);
    _ui_init_g_0_2();osDelay(SEND_INTERVAL_MS);
    _ui_init_g_0_3();osDelay(SEND_INTERVAL_MS);
    _ui_init_g_0_4();osDelay(SEND_INTERVAL_MS);
}

void ui_update_g_0() {
    _ui_update_g_0_0();osDelay(SEND_INTERVAL_MS);
    _ui_update_g_0_1();osDelay(SEND_INTERVAL_MS);
    _ui_update_g_0_2();osDelay(SEND_INTERVAL_MS);
    _ui_update_g_0_3();osDelay(SEND_INTERVAL_MS);
    _ui_update_g_0_4();osDelay(SEND_INTERVAL_MS);
}

void ui_remove_g_0() {
    _ui_remove_g_0_0();osDelay(SEND_INTERVAL_MS);
    _ui_remove_g_0_1();osDelay(SEND_INTERVAL_MS);
    _ui_remove_g_0_2();osDelay(SEND_INTERVAL_MS);
    _ui_remove_g_0_3();osDelay(SEND_INTERVAL_MS);
    _ui_remove_g_0_4();osDelay(SEND_INTERVAL_MS);
}

ui_5_frame_t ui_g_1_0;

ui_interface_rect_t *ui_g_1_Flywheel_Select = (ui_interface_rect_t*)&(ui_g_1_0.data[0]);
ui_interface_rect_t *ui_g_1_Spintop_Select = (ui_interface_rect_t*)&(ui_g_1_0.data[1]);
ui_interface_rect_t *ui_g_1_Autoaim_Select = (ui_interface_rect_t*)&(ui_g_1_0.data[2]);
ui_interface_number_t *ui_g_1_Supercap_Value = (ui_interface_number_t*)&(ui_g_1_0.data[3]);

void _ui_init_g_1_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_1_0.data[i].figure_name[0] = 0;
        ui_g_1_0.data[i].figure_name[1] = 1;
        ui_g_1_0.data[i].figure_name[2] = i + 0;
        ui_g_1_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_1_0.data[i].operate_type = 0;
    }

    ui_g_1_Flywheel_Select->figure_type = 1;
    ui_g_1_Flywheel_Select->operate_type = 1;
    ui_g_1_Flywheel_Select->layer = 1;
    ui_g_1_Flywheel_Select->color = 1;
    ui_g_1_Flywheel_Select->start_x = 1485;
    ui_g_1_Flywheel_Select->start_y = 800;
    ui_g_1_Flywheel_Select->width = 3;
    ui_g_1_Flywheel_Select->end_x = 1705;
    ui_g_1_Flywheel_Select->end_y = 850;

    ui_g_1_Spintop_Select->figure_type = 1;
    ui_g_1_Spintop_Select->operate_type = 1;
    ui_g_1_Spintop_Select->layer = 1;
    ui_g_1_Spintop_Select->color = 1;
    ui_g_1_Spintop_Select->start_x = 1485;
    ui_g_1_Spintop_Select->start_y = 730;
    ui_g_1_Spintop_Select->width = 3;
    ui_g_1_Spintop_Select->end_x = 1705;
    ui_g_1_Spintop_Select->end_y = 780;

    ui_g_1_Autoaim_Select->figure_type = 1;
    ui_g_1_Autoaim_Select->operate_type = 1;
    ui_g_1_Autoaim_Select->layer = 1;
    ui_g_1_Autoaim_Select->color = 1;
    ui_g_1_Autoaim_Select->start_x = 1485;
    ui_g_1_Autoaim_Select->start_y = 660;
    ui_g_1_Autoaim_Select->width = 3;
    ui_g_1_Autoaim_Select->end_x = 1705;
    ui_g_1_Autoaim_Select->end_y = 710;

    ui_g_1_Supercap_Value->figure_type = 6;
    ui_g_1_Supercap_Value->operate_type = 1;
    ui_g_1_Supercap_Value->layer = 1;
    ui_g_1_Supercap_Value->color = 6;
    ui_g_1_Supercap_Value->start_x = 540;
    ui_g_1_Supercap_Value->start_y = 820;
    ui_g_1_Supercap_Value->width = 3;
    ui_g_1_Supercap_Value->font_size = 25;
    ui_g_1_Supercap_Value->number = 66;


    ui_proc_5_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_update_g_1_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_1_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_remove_g_1_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_1_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}


void ui_init_g_1() {
    _ui_init_g_1_0();osDelay(SEND_INTERVAL_MS);
}

void ui_update_g_1() {
    _ui_update_g_1_0();osDelay(SEND_INTERVAL_MS);
}

void ui_remove_g_1() {
    _ui_remove_g_1_0();osDelay(SEND_INTERVAL_MS);
}

