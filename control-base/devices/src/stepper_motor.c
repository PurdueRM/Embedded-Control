// #include "stepper_motor.h"

// #include <stdint.h>
// #include <stdlib.h>
// #include "user_math.h"
// #include <math.h>

// #include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_def.h"
// #include "uart.h"

// //this might need to go in a main
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_USART1_UART_Init(void);

// UART_HandleTypeDef huart1;
// Stepper_Motor_Handle_t *stepper_motor;
// HAL_StatusTypeDef status;

// uint8_t rx_buffer[8] = { 0 };
// uint8_t tx_buffer[8] = { 0 };

// HAL_Init();
// SystemClock_Config();
// MX_GPIO_Init();
// MX_USART1_UART_Init();

// UART_Instance_t *uart1_instance = UART_Register(&huart1, rx_buffer, 8, tx_buffer, 8);
// stepper_motor = Stepper_Motor_Init(uart1_instance);


// Stepper_Motor_Handle_t * Stepper_Motor_Init(UART_Instance_t *uart_instance) {
//     Stepper_Motor_Handle_t *motor_handle = (Stepper_Motor_Handle_t *) malloc(sizeof(Stepper_Motor_Handle_t));
//     if (motor_handle == NULL) {
//         return NULL;
//     }

//     motor_handle->tx_buffer_len = 8;
//     motor_handle->tx_buffer = (uint8_t *) malloc(sizeof(uint8_t) * motor_handle->tx_buffer_len);
//     if (motor_handle->tx_buffer == NULL) {
//         free(motor_handle);
//         return NULL;
//     }

//     motor_handle->uart_instance = uart_instance;
//     motor_handle->direction = 0; // default to clockwise
//     motor_handle->spread_cycle = 0; // default to stealthChop
//     motor_handle->micro_steps = 256; // default to 256 microsteps per full step
    
//     motor_handle->hold_current = 0; // default to 0 hold current
//     motor_handle->current = 32; // default to full current
//     motor_handle->hold_delay = 0; // default to no hold delay

//     motor_handle->microstep_velocity = 0; // default to 0 microsteps per second
//     motor_handle->velocity = 0; // default to 0 rpm
//     motor_handle->microstep_position = 0; // default to 0 microsteps
//     motor_handle->position = 0; // default to 0 radians

//     motor_handle->temperature = 0; // default to 0 temperature
//     motor_handle->error_code = 0; // default to no error
//     motor_handle->id = 0; // default to motor ID 0
//     motor_handle->send_ready = 1; // default to ready to send
//     return motor_handle;
// }

// void Stepper_Motor_CRC(uint8_t* datagram, uint8_t datagramLength)
// {
//  int i,j;
//  uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
//  uint8_t currentByte;
//  *crc = 0;
//  for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
//  currentByte = datagram[i]; // Retrieve a byte to be sent from Array
//     for (j=0; j<8; j++) {
//         if ((*crc >> 7) ^ (currentByte&0x01)){ // update CRC based result of XOR operation
//             *crc = (*crc << 1) ^ 0x07;
//         }
//         else {
//             *crc = (*crc << 1);
//         }
//     currentByte = currentByte >> 1;
//     } // for CRC bit
//  } // for message byte
// }
// void Write_Stepper_Gen_Config(Stepper_Motor_Handle_t *st_handle, bool Direction = 0, bool spread_cycle = 0)
// {
//     // 0x00 2 controls stealthChop vs spread_cycle, 3 controls direction,
//     st_handle->send_ready = 0;
//     st_handle->spread_cycle = spread_cycle;
//     st_handle->direction = Direction;

//     st_handle->tx_buffer[0] = SYNCBYTE;
//     st_handle->tx_buffer[1] = 0x00;
//     st_handle->tx_buffer[2] = 0x00<<1 | 1; // 0x00 is the register address, 1 is write access
//     st_handle->tx_buffer[3] = 00 | Direction | spread_cycle | 0000; 
//     st_handle->tx_buffer[4] = 0x00;
//     st_handle->tx_buffer[5] = 0x00;
//     st_handle->tx_buffer[6] = 0x00;
//     Stepper_Motor_CRC(st_handle->tx_buffer,8);

//     st_handle->send_ready = 1;

// }

// void Write_Stepper_Velocity_Config(Stepper_Motor_Handle_t *st_handle, uint8_t Hold_current = 0 ,uint8_t Motor_current = 32, bool Hold_Delay = 0)
// {
    

//     if (Motor_current > 32) Motor_current = 32; 
//     if (Hold_current > 32) Hold_current = 32;

//     st_handle->send_ready = 0;
//     st_handle->hold_current = Hold_current;
//     st_handle->current = Motor_current;
//     st_handle->hold_delay = Hold_Delay;

//     st_handle->tx_buffer[0] = SYNCBYTE;
//     st_handle->tx_buffer[1] = 0x00;
//     st_handle->tx_buffer[2] = 0x10<<1 | 1; // 0x10 is the register address, 1 is write access
//     st_handle->tx_buffer[3] = Hold_current; // hold current /32
//     st_handle->tx_buffer[4] = Motor_current; // motor run current /32
//     st_handle->tx_buffer[5] = Hold_Delay; // hold delay
//     st_handle->tx_buffer[6] = 0x00;
//     Stepper_Motor_CRC(st_handle->tx_buffer,8);

//     st_handle->send_ready = 1;

// }

// void Write_Stepper_Velocity(Stepper_Motor_Handle_t *st_handle, float Velocity)
// {
//     // Write the velocity to the stepper motor
//     // It gives the motor velocity in +-(2^23)-1 [µsteps / t]
//     microstepVelocity = (int32_t)(Velocity * st_handle->micro_steps); // Convert to microsteps

//     st_handle->send_ready = 0;
//     st_handle->microstep_velocity = microstepVelocity;
//     st_handle->velocity = Velocity;

//     st_handle->tx_buffer[0] = SYNCBYTE;
//     st_handle->tx_buffer[1] = 0x00;
//     st_handle->tx_buffer[2] = 0x22<<1 | 1; // 0x00 is the register address, 1 is write access
//     st_handle->tx_buffer[3] = (uint8_t)(microstep_velocity >> 24);
//     st_handle->tx_buffer[4] = (uint8_t)(microstep_velocity >> 16);
//     st_handle->tx_buffer[5] = (uint8_t)(microstep_velocity >> 8);
//     st_handle->tx_buffer[6] = (uint8_t)(microstep_velocity);
//     Stepper_Motor_CRC(st_handle->tx_buffer,8);

//     st_handle->send_ready = 1;
    
// }

// HAL_StatusTypeDef DDSM_Motor_Send(Stepper_Motor_Handle_t *st_handle, UART_Instance_t *uart_instance) {
//     HAL_StatusTypeDef status = HAL_ERROR;

//     if (uart_instance->transmit_finished && st_handle->send_ready) {
//         uart_instance->transmit_finished = 0;

//         memcpy(uart_instance->tx_buffer, st_handle->tx_buffer, st_handle->tx_buffer_len);
//         st_handle->send_ready = 0;

//         status = HAL_UART_Transmit_IT(uart_instance->uart_handle, uart_instance->tx_buffer, st_handle->tx_buffer_len);

//         if (status != HAL_OK) { // Unlock if failed
//             uart_instance->transmit_finished = 1; 
//         }
//     }
//     else {
//         status = HAL_BUSY;
//     }
//     return status;
// }

// void Stepper_Motor_Receive(Stepper_Motor_Handle_t *st_handle) {
//     UART_Instance_t *uart_instance = st_handle->uart_instance;
//     uart_instance->receive_finished = 0;
//     //if (!crc8_validate(uart_instance->rx_buffer, uart_instance->rx_buffer_len)) return; TODO Add CRC check
//     switch (uart_instance->rx_buffer[3]) {
//     case 0x00: 
//         break;
//     }
//     uart_instance->receive_finished = 1;
// }

// //from uart.c of OB2
// UART_Instance_t *UART_Register(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t rx_buffer_len, uint8_t *tx_buffer, uint16_t tx_buffer_len)
// {
//     g_uart1.uart_handle = huart;
//     g_uart1.rx_buffer = rx_buffer;
//     g_uart1.rx_buffer_len = rx_buffer_len;
//     g_uart1.tx_buffer = tx_buffer;
//     g_uart1.tx_buffer_len = tx_buffer_len;
//     g_uart1.receive_finished = 0;
//     g_uart1.transmit_finished = 1;

//     // Start up receiving interrupt loop
//     HAL_UART_Receive_IT(g_uart1.uart_handle,
//                         g_uart1.rx_buffer, 
//                         g_uart1.rx_buffer_len);

//     // Start up transmitting interrupt loop
//     HAL_UART_Transmit_IT(g_uart1.uart_handle, g_uart1.tx_buffer, g_uart1.tx_buffer_len);

//     return &g_uart1;
// }

// //all from main.c of OB2
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// static void MX_USART1_UART_Init(void)
// {

//   /* USER CODE BEGIN USART1_Init 0 */

//   /* USER CODE END USART1_Init 0 */

//   /* USER CODE BEGIN USART1_Init 1 */

//   /* USER CODE END USART1_Init 1 */
//   huart1.Instance = USART1;
//   huart1.Init.BaudRate = 115200;
//   huart1.Init.WordLength = UART_WORDLENGTH_8B;
//   huart1.Init.StopBits = UART_STOPBITS_1;
//   huart1.Init.Parity = UART_PARITY_NONE;
//   huart1.Init.Mode = UART_MODE_TX_RX;
//   huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//   if (HAL_UART_Init(&huart1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN USART1_Init 2 */

//   /* USER CODE END USART1_Init 2 */

// }
// static void MX_GPIO_Init(void)
// {
// /* USER CODE BEGIN MX_GPIO_Init_1 */
// /* USER CODE END MX_GPIO_Init_1 */

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOD_CLK_ENABLE();
//   __HAL_RCC_GPIOA_CLK_ENABLE();

// /* USER CODE BEGIN MX_GPIO_Init_2 */
// /* USER CODE END MX_GPIO_Init_2 */
// }
