/**
 * @file Referee_System.c
 * @author Leo Liu
 * @brief reading data from the referee system
 * @version 2.0
 * @date 2024-05-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "referee_system.h"

Referee_System_t Referee_System;
Referee_Robot_State_t Referee_Robot_State;
UART_Instance_t *g_referee_uart_instance_ptr;
Daemon_Instance_t *g_referee_daemon_instance_ptr;

void Referee_Set_Robot_State(void)
{
    if (Referee_System.Online_Flag)
    {
        Referee_Robot_State.Game_Type = Referee_System.Game_Status.Type;
        Referee_Robot_State.ID = Referee_System.Robot_State.ID;
        Referee_Robot_State.Level = Referee_System.Robot_State.Level;

        Referee_Robot_State.Cooling_Rate = Referee_System.Robot_State.Shooter_Cooling_Value;
        Referee_Robot_State.Heat_Max = Referee_System.Robot_State.Shooter_Heat_Max;
        Referee_Robot_State.Launch_Speed_Max = DEFAULT_STANDARD_LAUNCH_SPEED_MAX;
        Referee_Robot_State.Chassis_Power_Max = Referee_System.Robot_State.Chassis_Power_Max;

        Referee_Robot_State.Chassis_Power = Referee_System.Power_Heat.Chassis_Power;
        Referee_Robot_State.Power_Buffer = Referee_System.Power_Heat.Chassis_Power_Buffer;
        Referee_Robot_State.Shooter_Heat_1 = Referee_System.Power_Heat.Shooter_1_17mm_Heat;
        Referee_Robot_State.Shooter_Heat_2 = Referee_System.Power_Heat.Shooter_2_17mm_Heat;
        Referee_Robot_State.Shooting_Frequency = Referee_System.Shooter.Frequency;
        Referee_Robot_State.Shooting_Speed = Referee_System.Shooter.Speed;
        Referee_Robot_State.Shooter_Power_Output = Referee_System.Robot_State.Shooter_Power_Output;
    }
    else
    {
        #ifdef ROBOT_TYPE_STANDARD
            Referee_Robot_State.Cooling_Rate = DEFAULT_STANDARD_COOLING_RATE+(Referee_Robot_State.Manual_Level-1)*5;
            Referee_Robot_State.Heat_Max = DEFAULT_STANDARD_HEAT_MAX+(Referee_Robot_State.Manual_Level-1)*50;
            Referee_Robot_State.Launch_Speed_Max = DEFAULT_STANDARD_LAUNCH_SPEED_MAX;
            Referee_Robot_State.Chassis_Power = (DEFAULT_STANDARD_POWER_MAX-10)+(Referee_Robot_State.Manual_Level-1)*5;
            Referee_Robot_State.Chassis_Power_Max = DEFAULT_STANDARD_POWER_MAX+(Referee_Robot_State.Manual_Level-1)*5+g_supercap.supercap_enabled_flag*20.0f;
        #endif

        #ifdef ROBOT_TYPE_HERO
            Referee_Robot_State.Cooling_Rate = DEFAULT_HERO_COOLING_RATE+(Referee_Robot_State.Manual_Level-1)*8;
            Referee_Robot_State.Heat_Max = DEFAULT_HERO_HEAT_MAX+(Referee_Robot_State.Manual_Level-1)*30;
            Referee_Robot_State.Launch_Speed_Max = DEFAULT_HERO_LAUNCH_SPEED_MAX;
            Referee_Robot_State.Chassis_Power_Max = DEFAULT_HERO_POWER_MAX+(Referee_Robot_State.Manual_Level-1)*5;
        #endif

        #ifdef ROBOT_TYPE_SENTRY
            Referee_Robot_State.Cooling_Rate = DEFAULT_SENTRY_COOLING_RATE;
            Referee_Robot_State.Heat_Max = DEFAULT_SENTRY_HEAT_MAX;
            Referee_Robot_State.Launch_Speed_Max = DEFAULT_SENTRY_LAUNCH_SPEED_MAX;
            Referee_Robot_State.Chassis_Power_Max = DEFAULT_SENTRY_POWER_MAX;
        #endif
    }
}
void Referee_System_Timeout_Callback()
{
	// Attemp to reinitialize UART service
	UART_Service_Init(g_referee_uart_instance_ptr);
    Referee_System.Online_Flag = 0;
}

void Referee_System_Init(UART_HandleTypeDef *huart)
{
    Referee_System.huart = huart;
    Referee_Robot_State.Manual_Level = 1;

    g_referee_uart_instance_ptr = UART_Register(huart, Referee_System.Buffer, REFEREE_BUFFER_LEN, Referee_Get_Data);
	
	// register Daemon instance
	// timeout is defined in the header file
	uint16_t reload_value = REFEREE_TIMEOUT_MS / DAEMON_PERIOD;
	uint16_t initial_counter = reload_value;
	g_referee_daemon_instance_ptr = Daemon_Register(reload_value, initial_counter, Referee_System_Timeout_Callback);
}

// Get referee system data based on ID
void Referee_Get_Data(UART_Instance_t *uart_instance)
{
    UNUSED(uart_instance);
    for (int n = 0; n < REFEREE_BUFFER_LEN;)
    {
        if (Referee_System.Buffer[n] == REFEREE_FRAME_HEADER_START)
        {
            Daemon_Reload(g_referee_daemon_instance_ptr);
            Referee_System.Online_Flag = 1;
            Referee_System.Info_Update_Frame++;
            if(Referee_System.Info_Update_Frame > 100)
                Referee_System.Info_Update_Frame = 0;

            switch (Referee_System.Buffer[n + 5] | Referee_System.Buffer[n + 6] << 8)
            {
            case REFEREE_GAME_STATUS:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_GAME_STATUS_LEN))
                {
                    memcpy(&Referee_System.Game_Status, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_GAME_STATUS_LEN-9]));
                    n += REFEREE_GAME_STATUS_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_GAME_RESULT:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_GAME_RESULT_LEN))
                {
                    memcpy(&Referee_System.Game_Result, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_GAME_RESULT_LEN-9]));
                    n += REFEREE_GAME_RESULT_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_HP:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_HP_LEN))
                {
                    memcpy(&Referee_System.Alive_Robot, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_HP_LEN-9]));
                    n += REFEREE_ROBOT_HP_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_EVENT_DATA:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_EVENT_DATA_LEN))
                {
                    memcpy(&Referee_System.Event, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_EVENT_DATA_LEN-9]));
                    n += REFEREE_EVENT_DATA_LEN;
                }
                else
                    n++;
                break;
			case REFEREE_SUPPLY_STATION:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_SUPPLY_STATION_LEN))
                {
                    memcpy(&Referee_System.Supply_Station, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_SUPPLY_STATION_LEN-9]));
                    n += REFEREE_SUPPLY_STATION_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_REFEREE_WARNING:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_REFEREE_WARNING_LEN))
                {
                    memcpy(&Referee_System.Warning, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_REFEREE_WARNING_LEN-9]));
                    n += REFEREE_REFEREE_WARNING_LEN;
                }
                else
                    n++;
                break;
		    case REFEREE_DART_DATA:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_DART_DATA_LEN))
                {
                    memcpy(&Referee_System.Dart_Info, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_DART_DATA_LEN-9]));
                    n += REFEREE_DART_DATA_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_STATE:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_STATE_LEN))
                {
                    memcpy(&Referee_System.Robot_State, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_STATE_LEN-9]));
                    n += REFEREE_ROBOT_STATE_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_POWER_HEAT:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_POWER_HEAT_LEN))
                {
                    memcpy(&Referee_System.Power_Heat, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_POWER_HEAT_LEN-9]));
                    n += REFEREE_POWER_HEAT_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_POSITION:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_POSITION_LEN))
                {
                    memcpy(&Referee_System.Robot_Pos, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_POSITION_LEN-9]));
                    n += REFEREE_ROBOT_POSITION_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_BUFF:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_BUFF_LEN))
                {
                    memcpy(&Referee_System.Robot_Buff, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_BUFF_LEN-9]));
                    n += REFEREE_ROBOT_BUFF_LEN;
                }
                else
                    n++;
                break;
			case REFEREE_AERIAL_DATA:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_AERIAL_DATA_LEN))
                {
                    memcpy(&Referee_System.Aerial_Data, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_AERIAL_DATA_LEN-9]));
                    n += REFEREE_AERIAL_DATA_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_INJURY_STATE:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_INJURY_STATE_LEN))
                {
                    memcpy(&Referee_System.Injury_State, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_INJURY_STATE_LEN-9]));
                    n += REFEREE_INJURY_STATE_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_SHOOTER_STATE:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_SHOOTER_STATE_LEN))
                {
                    memcpy(&Referee_System.Shooter, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_SHOOTER_STATE_LEN-9]));
                    n += REFEREE_SHOOTER_STATE_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_REMAINING_AMMO:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_REMAINING_AMMO_LEN))
                {
                    memcpy(&Referee_System.Remaining_Ammo, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_REMAINING_AMMO_LEN-9]));
                    n += REFEREE_REMAINING_AMMO_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_RFID:
                if (Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_RFID_LEN))
                {
                    memcpy(&Referee_System.RFID, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_RFID_LEN-9]));
                    n += REFEREE_ROBOT_RFID_LEN;
                }
                else
                    n++;
                break;
            default:
                n++;
                break;
            }
        }
        else
            n++;
    }
}