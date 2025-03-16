/**
 * @file Referee_System.h
 * @author Leo Liu
 * @brief header file for referee system
 * @version 2.0
 * @date 2024-05-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __REFEREE_SYSTEM_H
#define __REFEREE_SYSTEM_H

#include "bsp_crc.h"
#include "dma.h"
#include "bsp_uart.h"
#include "bsp_daemon.h"
#include "supercap.h"
#include <stdint.h>
#include <stdbool.h>

#define REFEREE_TIMEOUT_MS 500

#define ROBOT_TYPE_STANDARD
//#define ROBOT_TYPE_HERO
//#define ROBOT_TYPE_SENTRY

//Standard Default Configuration
#define DEFAULT_STANDARD_POWER_MAX 45
#define DEFAULT_STANDARD_LAUNCH_SPEED_MAX 30
#define DEFAULT_STANDARD_HEAT_MAX 200
#define DEFAULT_STANDARD_COOLING_RATE 10

//Sentry Default Configuration
#define DEFAULT_SENTRY_POWER_MAX 100
#define DEFAULT_SENTRY_LAUNCH_SPEED_MAX 30
#define DEFAULT_SENTRY_HEAT_MAX 400
#define DEFAULT_SENTRY_COOLING_RATE 80

//Hero Default Configuration
#define DEFAULT_HERO_POWER_MAX 55
#define DEFAULT_HERO_LAUNCH_SPEED_MAX 16
#define DEFAULT_HERO_HEAT_MAX 200
#define DEFAULT_HERO_COOLING_RATE 40

#define REFEREE_BUFFER_LEN 							273u					//Buffer length to receive all data
#define REFEREE_FRAME_HEADER_START 					0xA5 				//Frame header
#define REFEREE_FRAME_HEADER_LEN 					5 						//Frame header length
#define REFEREE_ID_LEN 								2						//ID length
#define REFEREE_FRAME_TAIL_LEN 						2 						//Frame tail length

#define REFEREE_OFFSET_SOF         					0  					//SOF offset
#define REFEREE_OFFSET_DATA_LENGTH 					1  					//Data length offset
#define REFEREE_OFFSET_SEQ         					3  					//SEQ offset
#define REFEREE_OFFSET_CRC8        					4  					//CRC offset

#define       REFEREE_GAME_STATUS	               0x0001       //Competition status data, transmitted at a fixed frequency of 1 Hz.
#define       REFEREE_GAME_RESULT              	   0x0002 			//Competition result data, transmitted upon completion of the competition.
#define       REFEREE_ROBOT_HP                     0x0003 			//Robot health data, transmitted at a fixed frequency of 3 Hz.
#define       REFEREE_EVENT_DATA                   0x0101				//Site event data, transmitted at a fixed frequency of 1 Hz
#define       REFEREE_SUPPLY_STATION               0x0102				//Action identifier data of the Official Projectile Supplier, transmitted when the Official Projectile Supplier releases projectiles.
#define       REFEREE_REFEREE_WARNING              0x0104				//Referee warning data, transmitted when one's team is issued a penalty/forfeiture and at a fixed frequency of 1 Hz in other cases.
#define       REFEREE_DART_DATA               	   0x0105				//Dart launching data, transmitted at a fixed frequency of 1 Hz.
#define       REFEREE_ROBOT_STATE                  0x0201				//Robot performance system data, transmitted at a fixed frequency of 10 Hz.
#define       REFEREE_POWER_HEAT                   0x0202				//Real-time chassis power and barrel heat data, transmitted at a fixed frequency of 50 Hz
#define       REFEREE_ROBOT_POSITION               0x0203				//Robot position data, transmitted at a fixed frequency of 1 Hz.
#define       REFEREE_ROBOT_BUFF                   0x0204				//Robot buff data, transmitted at a fixed frequency of 3 Hz.
#define       REFEREE_AERIAL_DATA                  0x0205				//Air support time data, transmitted at a fixed frequency of 1 Hz.
#define       REFEREE_INJURY_STATE                 0x0206				//Damage status data, transmitted after the damage occurs.
#define       REFEREE_SHOOTER_STATE                0x0207				//Real-time launching data, transmitted after a projectile is launched.
#define       REFEREE_REMAINING_AMMO               0x0208				//Projectile allowance, transmitted at a fixed frequency of 10 Hz.
#define       REFEREE_ROBOT_RFID                   0x0209				//Robot RFID module status, transmitted at a fixed frequency of 3 Hz.

/*Calculation: REFEREE_FRAME_HEADER_LEN(5-byte) + REFEREE_ID_LEN(2-byte) + DATA(n-byte) + REFEREE_FRAME_TAIL_LEN(2-byte,CRC16)*/
#define       REFEREE_GAME_STATUS_LEN	           20
#define       REFEREE_GAME_RESULT_LEN              10 
#define       REFEREE_ROBOT_HP_LEN                 41 
#define       REFEREE_EVENT_DATA_LEN               13
#define       REFEREE_SUPPLY_STATION_LEN           13
#define       REFEREE_REFEREE_WARNING_LEN          12
#define       REFEREE_DART_DATA_LEN           	   12	
#define       REFEREE_ROBOT_STATE_LEN              22
#define       REFEREE_POWER_HEAT_LEN               25
#define       REFEREE_ROBOT_POSITION_LEN           25
#define       REFEREE_ROBOT_BUFF_LEN               15
#define       REFEREE_AERIAL_DATA_LEN              11
#define       REFEREE_INJURY_STATE_LEN             10
#define       REFEREE_SHOOTER_STATE_LEN            16
#define       REFEREE_REMAINING_AMMO_LEN           15
#define       REFEREE_ROBOT_RFID_LEN               13

typedef struct
{
	uint8_t Game_Type; //1 for 7v7, 4 for 3v3, 5 for 1v1
	uint8_t ID; //3,4,5 Red Standard - 103,104,105 Blue Standard
	uint8_t Level;
	uint8_t Manual_Level;
	
	uint16_t Cooling_Rate;
	uint16_t Heat_Max;
	uint16_t Launch_Speed_Max;
	uint16_t Chassis_Power_Max;
	
	float Chassis_Power;
	float Power_Buffer;
	uint16_t Shooter_Heat_1;
  	uint16_t Shooter_Heat_2;
	uint8_t Shooting_Frequency;
	float Shooting_Speed;
	uint8_t Shooter_Power_Output;
}Referee_Robot_State_t;

typedef struct
{
	UART_HandleTypeDef *huart;
	uint16_t DMA_Counter;
	uint8_t Buffer[REFEREE_BUFFER_LEN];
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Type:4;
		uint8_t Progress:4;
		uint16_t Remaining_Time;
		uint64_t Sync_Time_Stamp;
	}Game_Status;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Winner;
	}Game_Result;
	
	struct __attribute__ ((__packed__))
	{
		uint16_t Red_1_HP;   
		uint16_t Red_2_HP;  
		uint16_t Red_3_HP;  
		uint16_t Red_4_HP;  
		uint16_t Red_5_HP;  
		uint16_t Red_7_HP;  
		uint16_t Red_Outpost_HP; 
		uint16_t Red_Base_HP; 
		
		uint16_t Blue_1_HP;   
		uint16_t Blue_2_HP;   
		uint16_t Blue_3_HP;   
		uint16_t Blue_4_HP;   
		uint16_t Blue_5_HP;   
		uint16_t Blue_7_HP; 
		uint16_t Blue_Outpost_HP; 
		uint16_t Blue_Base_HP;   
	}Alive_Robot;
	
	struct __attribute__ ((__packed__))
	{
		uint32_t Event_Data;
	}Event;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Reserved;
		uint8_t Supply_Robot_ID;
		uint8_t Supply_Projectile_Step;
		uint8_t Supply_Projectile_Num;
	}Supply_Station;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Card;
		uint8_t Foul_Robot_ID;
		uint8_t Count;
	}Warning;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Dart_Remaining_Time;
		uint16_t Dart_Info;
	}Dart_Info;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t ID;
		uint8_t Level;
		uint16_t Remaining_HP;
		uint16_t HP_Max;
		uint16_t Shooter_Cooling_Value;
		uint16_t Shooter_Heat_Max;
		uint16_t Chassis_Power_Max;	
		uint8_t Gimbal_Power_Output:1;
		uint8_t Chassis_Power_Output:1;
		uint8_t Shooter_Power_Output:1;
	}Robot_State;
	
	struct __attribute__ ((__packed__))
	{
		uint16_t Chassis_Voltage;
		uint16_t Chassis_Current;
		float Chassis_Power;
		uint16_t Chassis_Power_Buffer;
		uint16_t Shooter_1_17mm_Heat;
		uint16_t Shooter_2_17mm_Heat;
		uint16_t Shooter_42mm_Heat;
	}Power_Heat;
	
	struct __attribute__ ((__packed__))
	{
		float x;
		float y;
		float Angle;
	}Robot_Pos;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Recovery_Buff;
		uint8_t Cooling_Buff;
		uint8_t Defence_Buff;
		uint8_t Vulnerability_Buff;
		uint16_t Attack_Buff;
	}Robot_Buff;
	
	struct __attribute__ ((__packed__))
	{
	 uint8_t Aerial_Status;
	 uint8_t Time_Remaining;
	}Aerial_Data;
	
	struct __attribute__ ((__packed__))
	{
	 uint8_t Armor_ID : 4;
	 uint8_t HP_Deduction_Reason : 4;
	}Injury_State;
	
	struct __attribute__ ((__packed__))
	{
		uint8_t Ammo_Type;
		uint8_t Shooter_Type;
		uint8_t Frequency;
		float Speed;
	}Shooter;
	
	struct __attribute__ ((__packed__))
	{
		uint16_t Type_17mm;
		uint16_t Type_42mm;
		uint16_t Type_Gold;
	}Remaining_Ammo;
	
	struct __attribute__ ((__packed__))
	{
		uint32_t State;
	}RFID;
	
	uint8_t Info_Update_Frame;
	uint8_t Online_Flag;
} Referee_System_t;

void Referee_Get_Data(UART_Instance_t *uart_instance);
void Referee_Set_Robot_State(void);
void Referee_System_Init(UART_HandleTypeDef *huart);

extern Referee_Robot_State_t Referee_Robot_State;
extern Referee_System_t Referee_System;

#endif
