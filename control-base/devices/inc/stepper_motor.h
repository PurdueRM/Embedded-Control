// #ifndef STEPPER_MOTOR_H
// #define STEPPER_MOTOR_H

// #include <stdint.h>

// #define SYNCBYTE (10100000) //1010 plus reserved



// typedef struct DDSM_Motor {
//     UART_Instance_t *uart_instance;

//     uint16_t tx_buffer_len;
//     uint8_t *tx_buffer;

//     bool direction; // 0 for clockwise, 1 for counterclockwise
//     bool spread_cycle; // 0 for stealthChop, 1 for spreadCycle
//     int8_t micro_steps; // microsteps per full step


//     uint8_t current; // as a fraction of 32, so 32 is full current
//     uint8_t hold_current; // as a fraction of 32, so 32 is full current
//     uint8_t hold_delay; // 0 for no hold delay, else 2^18 clock cycles*value

//     float velocity; // rpm
//     int32_t microstep_velocity; // microsteps per second

//     float position; // radians
//     int32_t microstep_position; // microsteps

//     uint8_t temperature; // no need to convert
//     uint8_t error_code;

//     uint8_t id; // motor ID

//     uint8_t send_ready; // flag to know if tx buffer is ready

// } Stepper_Motor_Handle_t;

// Stepper_Motor_Handle_t* Stepper_Motor_Init(UART_HandleTypeDef *huart);
// void Stepper_Motor_CRC(uint8_t *data, uint8_t len);
// void Write_Stepper_Gen_Config(Stepper_Motor_Handle_t *st_handle, bool Direction, bool spread_cycle);
// void Write_Stepper_Velocity_Config(Stepper_Motor_Handle_t *st_handle, uint8_t Hold_current, uint8_t Motor_current, bool Hold_Delay);
// HAL_StatusTypeDef DDSM_Motor_Send(Stepper_Motor_Handle_t *st_handle, UART_Instance_t *uart_instance);
// void Stepper_Motor_Receive(Stepper_Motor_Handle_t *st_handle);  
// UART_Instance_t *UART_Register(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t rx_buffer_len, uint8_t *tx_buffer, uint16_t tx_buffer_len);
// //void SystemClock_Config(void);
// //static void MX_USART1_UART_Init(void);
// //static void MX_GPIO_Init(void);
