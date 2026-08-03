#include "jetson_orin.h"

#include <string.h>
#include <math.h>
#include "imu_task.h"
#include "bsp_daemon.h"
#include  "omni_locomotion.h"

extern pose_2d_t sentry_pose;
Jetson_Orin_Data_t g_orin_data;

UART_Instance_t *g_orin_uart_instance_ptr;
Daemon_Instance_t *g_orin_daemon_instance_ptr, *g_orin_daemon_nav_instance_ptr;
uint8_t g_jetson_orin_initialized;

void Jetson_Orin_Rx_Callback(UART_Instance_t *uart_instance)
{
	UNUSED(uart_instance);
	// TODO: update buffer reference from uart_instance
	if (g_orin_data.rx_buffer[0] == 0xAA)
	{
		// If matching frame header, reload Daemon
		Daemon_Reload(g_orin_daemon_instance_ptr);

		// Decode data
		g_orin_data.receiving.frame_id = g_orin_data.rx_buffer[0];
		g_orin_data.receiving.frame_type = g_orin_data.rx_buffer[1];
		// ! TODO make an ENUM!!!!!
		switch (g_orin_data.receiving.frame_type)
		{
		case 0:
			memcpy(&g_orin_data.receiving.float_byte.data[0], &g_orin_data.rx_buffer[4], 12 * sizeof(uint8_t));
			g_orin_data.receiving.auto_aiming.yaw = g_orin_data.receiving.float_byte.data[0];
			g_orin_data.receiving.auto_aiming.pitch = g_orin_data.receiving.float_byte.data[1];
			if (isnan(g_orin_data.receiving.auto_aiming.yaw) || isnan(g_orin_data.receiving.auto_aiming.pitch))
			{
				g_orin_data.receiving.auto_aiming.yaw = 0;
				g_orin_data.receiving.auto_aiming.pitch = 0;
			}
			g_orin_data.receiving.auto_aiming.fire = g_orin_data.receiving.float_byte.data_bytes[8];
			g_orin_data.new_data_flag = 1;
			break;

		case 1:
			Daemon_Reload(g_orin_daemon_nav_instance_ptr);
			memcpy(&g_orin_data.receiving.float_byte.data[0], &g_orin_data.rx_buffer[4], 12 * sizeof(uint8_t));
			g_orin_data.receiving.navigation.x_vel = g_orin_data.receiving.float_byte.data[0];
			g_orin_data.receiving.navigation.y_vel = g_orin_data.receiving.float_byte.data[1];
			g_orin_data.receiving.navigation.yaw_angular_rate = g_orin_data.receiving.float_byte.data[2];
			g_orin_data.receiving.navigation.state = g_orin_data.rx_buffer[16];
			break;
		case 2:
			g_orin_data.receiving.heart_beat.a = g_orin_data.rx_buffer[2];
			g_orin_data.receiving.heart_beat.b = g_orin_data.rx_buffer[3];
			g_orin_data.receiving.heart_beat.c = g_orin_data.rx_buffer[4];
			g_orin_data.receiving.heart_beat.d = g_orin_data.rx_buffer[5];
			break;

		default:
			break;
		}
	}
}

/**
 * @brief Callback function for timeout of Jetson Orin used by Daemon
 * @details When not receiving data from Orin for a certain time, will try
 * 			to reinitialize the UART service. Daemon is reloaded to prevent
 * 			continuous reinitialization at high frequency.
 * @param void
 */
void Jetson_Orin_Timeout_Callback() // this is dangerous w/ circular DMA (must change)
{
	// Attemp to reinitialize UART service
	UART_Service_Init(g_orin_uart_instance_ptr);
}

void Jetson_Orin_Nav_Timeout_Callback()
{
	g_orin_data.receiving.navigation.x_vel = 0.0f;
	g_orin_data.receiving.navigation.y_vel = 0.0f;
	g_orin_data.receiving.navigation.yaw_angular_rate = 0.0f;
}

void Jetson_Orin_Init(UART_HandleTypeDef *huartx)
{
	// register UART instance
	g_orin_uart_instance_ptr = UART_Register(huartx, g_orin_data.rx_buffer, ORIN_DATA_RX_BUFER_SIZE, Jetson_Orin_Rx_Callback);

	// register Daemon instance
	// timeout is defined in the header file
	uint16_t reload_value = ORIN_TIMEOUT_MS / DAEMON_PERIOD;
	uint16_t initial_counter = reload_value;
	g_orin_daemon_instance_ptr = Daemon_Register(reload_value, initial_counter, Jetson_Orin_Timeout_Callback);

	reload_value = ORIN_NAV_TIMEOUT_MS / DAEMON_PERIOD;
	initial_counter = reload_value;
	g_orin_daemon_nav_instance_ptr = Daemon_Register(reload_value, initial_counter, Jetson_Orin_Nav_Timeout_Callback);
	g_jetson_orin_initialized = 1;
}

void Jetson_Orin_Send_Data(void)
{
	if (!g_jetson_orin_initialized) {
		return;
	}

	// update data to be sent
	g_orin_data.sending.header = 0xAA; // header byte
	g_orin_data.sending.enemy_color_is_red = (Referee_System.Robot_State.ID > 11) ? 1 : 0;	// ID > 11 means myself is blue, which means enemy is red
	g_orin_data.sending.game_status = Referee_System.Game_Status.Progress;
	g_orin_data.sending.rfid = (Referee_System.RFID.center_zone << 1) | (Referee_System.RFID.resupply_zone); // bit 0 for resupply, bit 1 for center zone
	g_orin_data.sending.pitch_angle = g_imu.rad.pitch;
	g_orin_data.sending.pitch_angular_rate = g_imu.bmi088_raw.gyro[1];
	g_orin_data.sending.yaw_angular_rate = g_imu.bmi088_raw.gyro[2];
	g_orin_data.sending.position_x = sentry_pose.y;
	g_orin_data.sending.position_y = -sentry_pose.x;
	g_orin_data.sending.orientation = g_imu.rad.yaw;
	g_orin_data.sending.velocity_x = sentry_pose.vy;
	g_orin_data.sending.velocity_y = -sentry_pose.vx;
	g_orin_data.sending.HP = Referee_System.Robot_State.Remaining_HP;
	g_orin_data.sending.reserved = 7529; // reserved for future use

	// // float to byte conversion
	// g_orin_data.sending.float_byte.data[0] = g_orin_data.sending.pitch_angle;
	// g_orin_data.sending.float_byte.data[1] = g_orin_data.sending.pitch_angular_rate;
	// g_orin_data.sending.float_byte.data[2] = g_orin_data.sending.yaw_angular_rate;
	// g_orin_data.sending.float_byte.data[3] = g_orin_data.sending.position_x;
	// g_orin_data.sending.float_byte.data[4] = g_orin_data.sending.position_y;
	// g_orin_data.sending.float_byte.data[5] = g_orin_data.sending.orientation;
	// g_orin_data.sending.float_byte.data[6] = g_orin_data.sending.velocity_x;
	// g_orin_data.sending.float_byte.data[7] = g_orin_data.sending.velocity_y;
	// g_orin_data.sending.float_byte.data[8] 

	// g_orin_data.tx_buffer[0] = 0xAA;
	// g_orin_data.tx_buffer[1] = 0;
	// g_orin_data.tx_buffer[1] = g_orin_data.sending.enemy_color_is_red << 7 | (g_orin_data.sending.game_status << g_orin_data.sending.game_status);
	// memcpy(&g_orin_data.tx_buffer[2], &g_orin_data.sending.float_byte.data_bytes[0], 32 * sizeof(uint8_t));

	UART_Transmit(g_orin_uart_instance_ptr, &(g_orin_data.sending.header), sizeof(g_orin_data.sending), UART_DMA);
}