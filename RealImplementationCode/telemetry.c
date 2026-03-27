/*
 * telemetry.c
 *
 *  Created on: Jul 10, 2025
 *      Author: jon
 */
#include "telemetry.h"
#include "main.h" // For HAL functions and access to global variables/structs
#include "cybergear.h"

/*
 * This file requires access to several global variables defined in main.c
 * or other modules. We declare them here as 'extern' to inform the compiler
 * that they exist elsewhere.
 */

// From main.c (or included headers like cybergear.h)
extern CyberGear CyberGearMotorList[MAX_MOTORS];
extern float roll_esp32;
extern float current_motor1_out;
extern float current_motor2_out;
extern float theta_des_l_telemetry;
extern float theta_des_r_telemetry;
extern float xc_des_l;
extern float xc_des_r;
extern float desired_v_left;
extern float desired_v_right;


float theta_des_l_telemetry;
float theta_des_r_telemetry;

// Motor macros from main.c for convenience
#define MOTOR_CG_LF CyberGearMotorList[0]
#define MOTOR_CG_LB CyberGearMotorList[1]
#define MOTOR_CG_RF CyberGearMotorList[2]
#define MOTOR_CG_RB CyberGearMotorList[3]

// Static variable to track DMA status. 'static' makes it private to this file.
static volatile bool telemetry_tx_ready = true;

/**
  * @brief Calculates a simple 8-bit checksum.
  * @param data Pointer to the data buffer.
  * @param len Length of the data.
  * @retval The calculated checksum.
  */
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

/**
  * @brief Populates and sends a telemetry packet over the specified UART using DMA.
  * @param huart Pointer to a UART_HandleTypeDef structure (e.g., &huart3).
  */
void Send_Telemetry(UART_HandleTypeDef *huart) {
    // 1. Safety check for DMA
    if (!telemetry_tx_ready) {
        return;
    }

    // Static buffer to stay off the stack
    static TelemetryPacket_t tx_packet;

    // 2. FILL THE PAYLOAD (your existing variable assignments)
    tx_packet.payload.motor_angle_rf = MOTOR_CG_RF.angle;
    tx_packet.payload.motor_angle_rb = MOTOR_CG_RB.angle;
    tx_packet.payload.motor_angle_lf = MOTOR_CG_LF.angle;
    tx_packet.payload.motor_angle_lb = MOTOR_CG_LB.angle;

    tx_packet.payload.roll_angle     = roll_esp32;

    // 3. SET PACKET METADATA (The "Transparent" part)
    tx_packet.sof = TELEMETRY_SOF;
    tx_packet.len = sizeof(TelemetryPayload_t); // Automatically calculates payload size

    // 4. CALCULATE CHECKSUM
    // We calculate over the payload only to match the ESP32 logic
    tx_packet.checksum = calculate_checksum((uint8_t*)&tx_packet.payload, tx_packet.len);

    // 5. START DMA TRANSMISSION
    telemetry_tx_ready = false;

    // The total size is: 1 (SOF) + 1 (LEN) + PayloadSize + 1 (CRC)
    uint16_t total_frame_size = 1 + 1 + tx_packet.len + 1;

    HAL_UART_Transmit_DMA(huart, (uint8_t*)&tx_packet, total_frame_size);
}

/**
  * @brief Callback to be called from the main HAL_UART_TxCpltCallback.
  */
void Telemetry_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Check if the callback is from the telemetry UART (USART3).
    // This makes the function safe if other UARTs also use Tx complete interrupts.
    if (huart->Instance == USART3) {
        // The transmission is complete, so we set the flag to true,
        // allowing the next call to Send_Telemetry() to proceed.
        telemetry_tx_ready = true;
    }
}

