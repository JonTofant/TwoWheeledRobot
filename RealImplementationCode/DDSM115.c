/*
 * DDSM115.c
 *
 *  Created on: Mar 6, 2025
 *      Author: tofant
 */
// Todo globalise this

#include "DDSM115.h"
#include <string.h>  // for memcpy
#include "main.h"
#include "math.h"
#include <stdint.h>

float dt = 0.02;

// Motor Parameters
const float MOTOR_TORQUE_CONSTANT_KT = 0.75f; // Nm/A (DDSM115)

const uint32_t ENCODER_FULL_RANGE_COUNTS = 32768;
const float ENCODER_HALF_RANGE_COUNTS = 16384.0f;
const float COUNTS_TO_RADIANS_FACTOR_PHI = (2.0f * M_PI) / (float)ENCODER_FULL_RANGE_COUNTS;
const float RPM_TO_RAD_PER_SEC = (2.0f * M_PI) / 60.0f; // ≈ 0.104719755f


float DZ_RIGHT_POS = 0.04f;
float DZ_RIGHT_NEG = 0.04f;
float DZ_LEFT_POS  = 0.04f;
float DZ_LEFT_NEG  = 0.04f;

DDSM115 DDSM115MotorList[MAX_MOTORS_DDSM115] = {
	{ .motorID = 0x11, .target_angle = 0.0f, .min_angle = -6.283f, .max_angle = 6.283f, .errorFlag = true, .x =0, .x_dot =0, .x_ddot =0, .prev_x_dot = 0 },
	{ .motorID = 0x10, .target_angle = 0.0f, .min_angle = -6.283f, .max_angle = 6.283f, .errorFlag = true, .x = 0, .x_dot = 0, .x_ddot = 0, .prev_x_dot = 0}
};


uint8_t position_mode[10] = {
    0x01,  // Motor ID
    0xA0,  // Command code for position loop command
    0x00,  // High byte of target position (10000 in decimal: 0x2710)
    0x00,  // Low byte of target position
    0x00,  // Reserved/unused (acceleration, brake, etc.)
    0x00,  // Reserved
    0x00,  // Reserved
    0x00,  // Reserved
    0x00,  // Reserved
    0x03,   // CRC8 checksum
};


uint8_t compute_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void sendPositionCommand(uint8_t motorID, float angle_deg) {
    uint8_t command[10] = {0};
    uint16_t target_value = angleToValue(angle_deg);

    // Fill command packet according to documentation:
    // Byte 0: Motor ID, Byte 1: Command code (0x64 for drive command)
    command[0] = motorID;
    command[1] = 0x64; // Drive command code for position control

    // Bytes 2-3: 16-bit target position (big-endian)
    command[2] = (uint8_t)(target_value >> 8);   // High byte
    command[3] = (uint8_t)(target_value & 0xFF);   // Low byte

    // Bytes 4-8: Reserved (set to 0)
    command[4] = 0x00;
    command[5] = 0x00;
    command[6] = 0x00;
    command[7] = 0x00;
    command[8] = 0x00;

    // Byte 9: CRC8 checksum over bytes 0 to 8
    command[9] = compute_crc8(command, 9);

    // Set RS485 transceiver to transmit mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    // Transmit the command
    HAL_UART_Transmit(&huart5, command, 10, HAL_MAX_DELAY);
    // Return RS485 transceiver to receive mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}


uint16_t angleToValue(float angle_deg) {
    // Clamp the angle between 0 and 360
    if(angle_deg < 0.0f) {
        angle_deg = 0.0f;
    }
    if(angle_deg > 360.0f) {
        angle_deg = 360.0f;
    }
    // Map 0-360° to 0-32767 (note: 32767 is the maximum unsigned 16-bit value used)
    return (uint16_t)((angle_deg / 360.0f) * 32767.0f);
}


// --- Set Mode Function ---
// mode should be one of:
//    0x01 -> current loop
//    0x02 -> velocity loop (if needed)
//    0x03 -> position loop
void DDMS115setMode(uint8_t motorID, uint8_t mode) {
    uint8_t cmd[10] = {0};
    cmd[0] = motorID;     // Motor ID
    cmd[1] = 0xA0;        // Command code for mode switching
    // Bytes 2-8 are reserved (set to 0)
    cmd[9] = mode;        // Mode value (no CRC calculation for mode command)

    // Set RS485 transceiver to transmit mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
    HAL_Delay(10);  // Short delay to allow command processing
    // Return RS485 transceiver to receive mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}


// --- Set Current Function ---
// In current loop mode, the motor expects a signed 16-bit value representing current,
// where -32767 corresponds to -8 A and +32767 corresponds to +8 A.
void DDSM115setCurrent(uint8_t motorID, float current_amp) {
    // Clamp current_amp to the range [-8.0, 8.0]
    if (current_amp > 8.0f) {
        current_amp = 8.0f;
    } else if (current_amp < -8.0f) {
        current_amp = -8.0f;
    }

    // Convert desired current to a signed 16-bit value:
    // current_value = (current_amp / 8.0) * 32767
    int16_t current_value = (int16_t)((current_amp / 8.0f) * 32767.0f);

    uint8_t cmd[10] = {0};
    cmd[0] = motorID;     // Motor ID
    cmd[1] = 0x64;        // Drive command code (used for sending current/position/speed)

    // Bytes 2-3: current_value as signed 16-bit (big-endian)
    cmd[2] = (uint8_t)((uint16_t)current_value >> 8);  // High byte
    cmd[3] = (uint8_t)(current_value & 0xFF);          // Low byte

    // Bytes 4-8: Reserved (set to 0)
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;

    // Byte 9: CRC8 checksum calculated over bytes 0 through 8
    cmd[9] = compute_crc8(cmd, 9);

    // Set RS485 transceiver to transmit mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
    // Return RS485 transceiver to receive mode
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

// --- Change Motor ID Function ---
// This function changes the motor ID. The new ID should be unique and not conflict with other motors.

void DDSM115ChangeID(uint8_t motorID, uint8_t newID){
    uint8_t cmd[10] = {0};

    cmd[0] = 0xAA;     // Motor ID
    cmd[1] = 0x55;        // Drive command code (used for sending current/position/speed)

    // Bytes 2-3: current_value as signed 16-bit (big-endian)
    cmd[2] = 0x53;  // High byte
    cmd[3] = newID;          // Low byte

    // Bytes 4-8: Reserved (set to 0)
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;

    // Byte 9: CRC8 checksum calculated over bytes 0 through 8
    cmd[9] = 0x00;

    // Sending 5 time in a for loop

    for (int i = 0; i < 5; i++) {
		// Set RS485 transceiver to transmit mode
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart5, cmd, 10, HAL_MAX_DELAY);
		// Return RS485 transceiver to receive mode
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);  // Allow time for the command to be processed

	}
}


void update_ddsm115_state(DDSM115* motor, const uint8_t* Buffer, float wheel_radius)
{
    if (Buffer[0] != motor->motorID) return;

    // 1. Velocity
    uint16_t raw_velocity = ((uint16_t)Buffer[4] << 8) | Buffer[5];
    int16_t rpm = (int16_t)raw_velocity;
    motor->phi_dot_rad_s = -(float)rpm * RPM_TO_RAD_PER_SEC;
    motor->x_dot = motor->phi_dot_rad_s * wheel_radius;

    // 2. Position
    uint16_t current_raw_pos = ((uint16_t)Buffer[6] << 8) | Buffer[7];

    if (!motor->initialized) {
        motor->prev_raw_pos = current_raw_pos;

        float raw_angle = (float)current_raw_pos * (2.0f * M_PI / RAW_POS_MAX_COUNT);
        motor->phi_zero = raw_angle;
        motor->phi_zero_initialized = true;

        motor->phi_rad = 0.0f;
        motor->num_rotations = 0;
        motor->x = 0.0f;
        motor->x_ddot = 0.0f; // Initialize acceleration
        motor->prev_x_dot = motor->x_dot; // Store initial velocity
        motor->initialized = true;
        return;
    }

    // 3. Acceleration
    if (dt > 0.0f) { // Avoid division by zero
        motor->x_ddot = -((motor->x_dot - motor->prev_x_dot) / dt);
    } else {
        motor->x_ddot = 0.0f; // Set acceleration to zero if dt is invalid
    }

    // 4. Wrap detection
    int32_t delta = (int32_t)current_raw_pos - (int32_t)motor->prev_raw_pos;
    if (delta > RAW_POS_HALF_RANGE) motor->num_rotations--;
    else if (delta < -RAW_POS_HALF_RANGE) motor->num_rotations++;

    float raw_angle = (float)current_raw_pos * (2.0f * M_PI / RAW_POS_MAX_COUNT);
    float unwrapped = raw_angle + motor->num_rotations * 2.0f * M_PI;
    motor->phi_rad = unwrapped - motor->phi_zero;
    motor->x = motor->phi_rad * wheel_radius;

    // Update previous velocity
    motor->prev_x_dot = motor->x_dot;
    motor->prev_raw_pos = current_raw_pos;
}
