#include "ui_task.h"

void UI_Task_Loop(void)
{
    if(!g_robot_state.UI_ENABLED)
    {
        // If UI is not enabled, remove all UI elements
        ui_remove_g_0();
        ui_remove_g_1();
        
        // Then reinitialize the UI
        ui_init_g_0();
        ui_init_g_1();
        g_robot_state.UI_ENABLED = 1; // Mark UI as enabled
    }

    // Update flywheel selection box - yellow if flywheel is disabled, cyan if enabled
    if(g_robot_state.launch.IS_FLYWHEEL_ENABLED)
        ui_g_1_Spintop_Select->color = 6;
    else
        ui_g_1_Spintop_Select->color = 1;

    // Update spintop selection box - yellow if spintop is disabled, cyan if enabled
    if(g_robot_state.chassis.IS_SPINTOP_ENABLED)
        ui_g_1_Flywheel_Select->color = 6;
    else
        ui_g_1_Flywheel_Select->color = 1;
    
    // Update autoaim selection box - yellow if autoaim is disabled, cyan if enabled
    if(g_robot_state.launch.IS_AUTO_AIMING_ENABLED)
        ui_g_1_Autoaim_Select->color = 6;
    else
        ui_g_1_Autoaim_Select->color = 1;

    // Update supercapacitor value
    ui_g_1_Supercap_Value->number = g_supercap.supercap_percent;

    // Update the UI elements (only group 1 as they are dynamic elements)
    ui_update_g_1();
}