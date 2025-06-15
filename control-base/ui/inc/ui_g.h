//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"
#include "cmsis_os.h"

#define SEND_INTERVAL_MS (50)

extern ui_interface_line_t *ui_g_0_Path_R;
extern ui_interface_round_t *ui_g_0_Aim;
extern ui_interface_line_t *ui_g_0_Path_L;
extern ui_interface_string_t *ui_g_0_Flywheel_Text;
extern ui_interface_string_t *ui_g_0_Spintop_Text;
extern ui_interface_string_t *ui_g_0_Autoaim_Text;
extern ui_interface_string_t *ui_g_0_Supercap_Text;

void ui_init_g_0();
void ui_update_g_0();
void ui_remove_g_0();

extern ui_interface_rect_t *ui_g_1_Flywheel_Select;
extern ui_interface_rect_t *ui_g_1_Spintop_Select;
extern ui_interface_rect_t *ui_g_1_Autoaim_Select;
extern ui_interface_number_t *ui_g_1_Supercap_Value;

void ui_init_g_1();
void ui_update_g_1();
void ui_remove_g_1();


#endif // UI_g_H
