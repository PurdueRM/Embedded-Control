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
    __MAX_LIMIT(g_supercap.supercap_percent,0,100); // Ensure supercapacitor percentage is within 0-100%
    ui_g_1_Supercap_Percent->end_x = g_supercap.supercap_percent/100.0f * 640.0f + 640.0f; // Scale supercapacitor percentage to the UI width

    if(g_supercap.supercap_percent < 30)
        ui_g_1_Supercap_Percent->color = 5; // Set color to red if supercapacitor is below 30%
    else if (g_supercap.supercap_percent < 60)
        ui_g_1_Supercap_Percent->color = 3; // Set color to orange if supercapacitor is below 60%
    else 
        ui_g_1_Supercap_Percent->color = 2; // Set color to green if supercapacitor is above 60%
    
    // Update the UI elements (only group 1 as they are dynamic elements)
    ui_update_g_1();
}