/*
 * cybergear.c
 *
 *  Created on: Mar 5, 2025
 *      Author: tofan
 */


#include "cybergear.h"
#include <string.h>  // for memcpy
#include "main.h"
#include "math.h"

#define CURRENT_PARAM_INDEX 0x7006  // Parameter index for the current command (iq_ref)
#define ANGLE_PARAM_INDEX 0x7016    // Parameter index for the target angle
#define IQ_REF_PARAM_INDEX    0x7006  // iq_ref
#define OFFSET (0.4f)
#define leftFrontMinAngle M_PI_2-OFFSET
#define leftFrontMaxAngle (7 * M_PI/6)+OFFSET
#define leftBackMinAngle -M_PI/6-OFFSET
#define leftBackMaxAngle M_PI_2+OFFSET



uint8_t CAN_received_data[8];


// Initialize motors with errorFlag set to true (unverified) and names for clarity.
CyberGear CyberGearMotorList[MAX_MOTORS] = {
    { .hostID = 0xFE, .motorID = 0x1E, .angle = 1.0f, .min_angle = leftBackMinAngle, .max_angle = leftBackMaxAngle,
      .kp = 3.0f, .kd = 0.5f, .errorFlag = true, .max_velocity = 30.0f, .update_flag = false, .target_current_LQR = 0.0f, .desired_angle = 0.0f, .desired_velocity = 0.0f, .desired_torque_ff = 0.0f },
    { .hostID = 0xFE, .motorID = 0x1F, .angle = 1.0f, .min_angle = leftFrontMinAngle, .max_angle = leftFrontMaxAngle,
      .kp = 3.0f, .kd = 0.5f, .errorFlag = true,.max_velocity = 30.0f, .update_flag = false, .target_current_LQR = 0.0f, .desired_angle = 0.0f, .desired_velocity = 0.0f, .desired_torque_ff = 0.0f },
    { .hostID = 0xFE, .motorID = 0x15, .angle = 1.0f, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX,
      .kp = 3.0f, .kd = 0.5f, .errorFlag = true ,.max_velocity = 30.0f, .update_flag = false, .target_current_LQR = 0.0f, .desired_angle = 0.0f, .desired_velocity = 0.0f, .desired_torque_ff = 0.0f },
    { .hostID = 0xFE, .motorID = 0x14, .angle = 1.0f, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX,
      .kp = 3.0f, .kd = 0.5f, .errorFlag = true,.max_velocity = 30.0f, .update_flag = false, .target_current_LQR = 0.0f, .desired_angle = 0.0f, .desired_velocity = 0.0f, .desired_torque_ff = 0.0f }
};


// Send the current angle command for a motor via CAN.
void Motor_SendAngle(CyberGear* motor) {
    // Ensure the angle is within limits.
    if(motor->angle < motor->min_angle)
        motor->angle = motor->min_angle;
    else if(motor->angle > motor->max_angle)
        motor->angle = motor->max_angle;

    // Use your writeParameter function to send the angle.
    writeParameter(0x7016, &motor->target_angle, motor->hostID, motor->motorID);
}

/**
 * @brief Sends the desired current command over CAN to the motor.
 *
 * This function updates the motor's target current and sends it via the writeParameter
 * function using the CURRENT_PARAM_INDEX. It assumes the motor's current control loop
 * uses the parameter at index 0x7006 (e.g., iq_ref).
 *
 * @param motor Pointer to the CyberGear structure for the motor.
 * @param current The desired current command (in Amperes).
 * @return HAL_StatusTypeDef The status returned by the writeParameter call.
 */
HAL_StatusTypeDef Motor_SendCurrent(CyberGear* motor, float current) {
    // Optional: Add any safety or limit checks on 'current' here.
    // For example, you might ensure that current is within a predefined range.

    // Update the target current in the motor structure.
    motor->target_current = current;

    // Send the current command via CAN using the defined parameter index.
    return writeParameter(CURRENT_PARAM_INDEX, &motor->target_current, motor->hostID, motor->motorID);
}

// Request device ID from each motor.
void Motor_RequestDeviceIDs(void) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        getMotorDeviceID(CyberGearMotorList[i].hostID, CyberGearMotorList[i].motorID);
    }
}



HAL_StatusTypeDef writeParameter(uint16_t paramIndex, const volatile void* paramValue,
                                 uint8_t hostID, uint8_t motorID)

{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // Build extended ID => (type=18)
    uint32_t extId = ((uint32_t)0x12 << 24)
                   | ((uint32_t)hostID << 8)
                   | (uint32_t)motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // Byte0..1 = paramIndex (little-endian)
    txData[0] = (uint8_t)(paramIndex & 0xFF);
    txData[1] = (uint8_t)(paramIndex >> 8);
    // Byte2..3 = 0
    // Byte4..7 = paramValue
    memcpy(&txData[4], paramValue, 4);

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

HAL_StatusTypeDef clearMotorFault(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // 4 in bits28..24 => Stop command
    uint32_t extId = ((uint32_t)4 << 24)
                   | ((uint32_t)hostID << 8)
                   | (uint32_t)motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;     // extended frame
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // data[0] = 1 => clear fault
    txData[0] = 1;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}


HAL_StatusTypeDef motorEnable(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    uint32_t extId = ((uint32_t)3 << 24) |
                     ((uint32_t)hostID << 8) |
                     motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

HAL_StatusTypeDef getMotorDeviceID(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // 0 in bits28..24 => get device ID
    uint32_t extId = ((uint32_t)0 << 24)
                   | ((uint32_t)hostID << 8)
                   | (uint32_t)motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;     // extended frame
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // Typically data can be all 0
    memset(txData, 0, 8);

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

HAL_StatusTypeDef setMechanicalZero(uint8_t hostID, uint8_t motorID) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    // Communication type 6 is used to set mechanical zero.
    // Build the extended ID: type (6) is in bits28..24,
    // hostID is in bits15..8 and motorID in bits7..0.
    uint32_t extId = ((uint32_t)6 << 24) | ((uint32_t)hostID << 8) | motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // Set data byte 0 to 1 to indicate zeroing the angle.
    txData[0] = 1;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}



uint16_t float_to_uint(float x, float x_min, float x_max)
{
    // Clamp x into [x_min, x_max]
    if(x < x_min) x = x_min;
    if(x > x_max) x = x_max;

    float span = (x_max - x_min);
    float offset = (x - x_min);

    // Scale into [0..65535]
    // The Xiaomi snippet does:
    //   (x - offset)*(65535.0f)/span
    // but we can do something like:
    return (uint16_t)((offset * 65535.0f) / span + 0.5f);
}

// Function for single parameter read
HAL_StatusTypeDef singleParameterRead(uint16_t paramIndex, uint8_t hostID, uint8_t motorID)
{
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;
	uint8_t txData[8] = {0};

	// Build extended ID => (type=17) 0x11 = 17
	uint32_t extId = ((uint32_t)0x11 << 24)
				   | ((uint32_t)hostID << 8)
				   | (uint32_t)motorID;

	txHeader.ExtId = extId;
	txHeader.IDE   = CAN_ID_EXT;
	txHeader.RTR   = CAN_RTR_DATA;
	txHeader.DLC   = 8;
	txHeader.TransmitGlobalTime = DISABLE;

	// Byte0..1 = paramIndex (little-endian)
	txData[0] = (uint8_t)(paramIndex & 0xFF);
	txData[1] = (uint8_t)(paramIndex >> 8);
	// Byte2..7 = 0

	return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// Function for setIqRef()
HAL_StatusTypeDef setIqRef(CyberGear* motor, float current)
{
    // Optionally clamp the current to safe bounds, e.g. -23A..+23A
    if (current > 7.0f)  current = 7.0f;
    if (current < -7.0f) current = -7.0f;
    // Check if motor is in error state if it is set current to zero
    if (motor->errorFlag) current = 0.0f;

    // Store for reference in the motor structure
    motor->target_current = current;

    // Write to the IQ_REF_PARAM_INDEX (0x7006)
    return writeParameter(IQ_REF_PARAM_INDEX, &motor->target_current,
                          motor->hostID, motor->motorID);
}

// Function for motor stop

HAL_StatusTypeDef motorStop(CyberGear* motor) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // Build extended ID for Motor stop command (communication type 4)
    // The extended ID is built by shifting 4 (command type) into bits 28-24,
    // then including the hostID and motorID.
    uint32_t extId = ((uint32_t)4 << 24)
                   | ((uint32_t)motor->hostID << 8)
                   | (uint32_t)motor->motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // For a normal stop command, all data bytes are set to zero.
    memset(txData, 0, sizeof(txData));

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/**
 * @brief Sends MIT Mode Control command (Communication Type 1)
 * @param motor Pointer to motor struct containing target states and gains
 * @param torque_ff Feed-forward torque in Nm
 */
HAL_StatusTypeDef Motor_SendMITCommand(CyberGear* motor) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8];

    // 1. Calculate the 16-bit values
    uint16_t q_p  = float_to_uint(motor->desired_angle, -4.0f * M_PI, 4.0f * M_PI);
    uint16_t q_v  = float_to_uint(motor->desired_velocity, -30.0f, 30.0f);
    uint16_t q_kp = float_to_uint(motor->kp, 0.0f, 500.0f);
    uint16_t q_kd = float_to_uint(motor->kd, 0.0f, 5.0f);
    uint16_t q_t  = float_to_uint(motor->desired_torque_ff, -12.0f, 12.0f);

    // 2. Build the Extended ID carefully
    // Bit 28-24: Comm Mode (1)
    // Bit 23-8:  Torque (q_t)
    // Bit 7-0:   Motor ID
    uint32_t extId = 0;
    extId |= (uint32_t)0x01 << 24;                 // Mode 1
    extId |= (uint32_t)(q_t & 0xFFFF) << 8;        // Force 16-bit limit then shift
    extId |= (uint32_t)(motor->motorID & 0xFF);    // Force 8-bit limit

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // 3. Pack Data - Xiaomi expects Big Endian (MSB first)
    txData[0] = (uint8_t)(q_p >> 8);
    txData[1] = (uint8_t)(q_p & 0xFF);
    txData[2] = (uint8_t)(q_v >> 8);
    txData[3] = (uint8_t)(q_v & 0xFF);
    txData[4] = (uint8_t)(q_kp >> 8);
    txData[5] = (uint8_t)(q_kp & 0xFF);
    txData[6] = (uint8_t)(q_kd >> 8);
    txData[7] = (uint8_t)(q_kd & 0xFF);

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}
